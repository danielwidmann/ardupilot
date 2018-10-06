/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2
#include "OpticalFlow_RaspberryPi.h"

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "AP_HAL/utility/RingBuffer.h"

#include <cstdlib>

#define OPTICAL_FLOW_ONBOARD_RTPRIO 11
static const unsigned int OPTICAL_FLOW_GYRO_BUFFER_LEN = 400;

/* focal length 2.21mm pixel size 3.6 um, 2x binning in each direction
 * 240x240 crop rescaled to 64x64 */
#define HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX (2.21 / (3.6 * 2.0 * 240 / 64))

extern const AP_HAL::HAL& hal;

using namespace Linux;


class Linux::GyroSample {
public:
    Vector2f gyro;
    uint64_t time_us;
};


static uint64_t get_timestamp_us()
{
    struct timespec ts;
    uint64_t time_us;


    clock_gettime(CLOCK_MONOTONIC, &ts);

    time_us = 1.0e6 * (ts.tv_sec + (ts.tv_nsec*1.0e-9));

    return time_us;
}

void OpticalFlow_RaspberryPi::init()
{
    uint32_t top, left;
    uint32_t crop_width, crop_height;

    unsigned int nbufs = 0;
    int ret;
    pthread_attr_t attr;
    struct sched_param param = {
        .sched_priority = OPTICAL_FLOW_ONBOARD_RTPRIO
    };

    if (_initialized) {
        return;
    }


    mkfifo("/dev/shm/video_stream", 0666);

	system("raspivid -o /dev/null -x /dev/shm/video_stream -w 320 -h 320 -fps 90 -g 0 -fl -t 0 &");


	fp = fopen("/dev/shm/video_stream", "rb");

    /* Create the thread that will be waiting for frames
     * Initialize thread and mutex */
    ret = pthread_mutex_init(&_mutex, nullptr);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to init mutex");
    }

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to init attr");
    }
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    ret = pthread_create(&_thread, &attr, _read_thread, this);
    if (ret != 0) {
        AP_HAL::panic("OpticalFlow_Onboard: failed to create thread");
    }

    _gyro_ring_buffer = new ObjectBuffer<GyroSample>(OPTICAL_FLOW_GYRO_BUFFER_LEN);

    _initialized = true;
}

bool OpticalFlow_RaspberryPi::read(AP_HAL::OpticalFlow::Data_Frame& frame)
{
    bool ret;

    pthread_mutex_lock(&_mutex);
    if (!_data_available) {
        ret = false;
        goto end;
    }
    frame.pixel_flow_x_integral = _pixel_flow_x_integral;
    frame.pixel_flow_y_integral = _pixel_flow_y_integral;
    frame.gyro_x_integral = _gyro_x_integral;
    frame.gyro_y_integral = _gyro_y_integral;
    frame.delta_time = _integration_timespan;
    frame.quality = _surface_quality;
    _integration_timespan = 0;
    _pixel_flow_x_integral = 0;
    _pixel_flow_y_integral = 0;
    _gyro_x_integral = 0;
    _gyro_y_integral = 0;
    _data_available = false;
    ret = true;
end:
    pthread_mutex_unlock(&_mutex);
    return ret;
}

void OpticalFlow_RaspberryPi::push_gyro(float gyro_x, float gyro_y, float dt)
{
    GyroSample sample;
    struct timespec ts;

    if (!_gyro_ring_buffer) {
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &ts);
    _integrated_gyro.x += (gyro_x - _gyro_bias.x) * dt;
    _integrated_gyro.y += (gyro_y - _gyro_bias.y) * dt;
    sample.gyro = _integrated_gyro;
    sample.time_us = 1.0e6 * (ts.tv_sec + (ts.tv_nsec*1.0e-9));

    _gyro_ring_buffer->push(sample);
}

void OpticalFlow_RaspberryPi::_get_integrated_gyros(uint64_t timestamp, GyroSample &gyro)
{
    GyroSample integrated_gyro_at_time = {};
    unsigned int retries = 0;

    // pop all samples prior to frame time
    while (_gyro_ring_buffer->pop(integrated_gyro_at_time) &&
            integrated_gyro_at_time.time_us < timestamp &&
            retries++ < OPTICAL_FLOW_GYRO_BUFFER_LEN);
    gyro = integrated_gyro_at_time;
}

void OpticalFlow_RaspberryPi::push_gyro_bias(float gyro_bias_x, float gyro_bias_y)
{
    _gyro_bias.x = gyro_bias_x;
    _gyro_bias.y = gyro_bias_y;
}

void *OpticalFlow_RaspberryPi::_read_thread(void *arg)
{
    OpticalFlow_RaspberryPi *optflow_raspberrypi = (OpticalFlow_RaspberryPi *) arg;

    optflow_raspberrypi->_run_optflow();
    return nullptr;
}

void OpticalFlow_RaspberryPi::_calculate_flow(float* out_x, float* out_y, uint64_t* timestamp)
{
	const int width = 320;
	const int height = 320;
	const int cols = ((width + 15) / 16) + 1;
	const int rows = (height + 15) / 16;


	const int number_elements = rows * cols;

	uint32_t buffer[number_elements];

	int8_t x_values[number_elements];
	int8_t y_values[number_elements];

	int bytes_read = fread(buffer, 4/*sizeof(element_t)*/, number_elements, fp);

	if (bytes_read == 0) {
		// todo panic
		return;
	}

	if (bytes_read != number_elements) {
		// incomplete frame
		return;
	}

	*timestamp = get_timestamp_us();

	int idx = 0;
	// do not use outermost motion vectors as they are generally not very good
	// (It is impossible to estimate motion of an macroblock that is moving into the camera)
	for (int row_idx = 1; row_idx < rows - 2; row_idx++)
	{
		for (int col_idx = 1; col_idx < cols - 1 - 2; col_idx++)
		{
			uint32_t value = buffer[col_idx * rows + row_idx];
			x_values[idx] = value;//value >> 24;
			y_values[idx] = value >> 8;//value >> 16;

			idx++;
		}
	}

	int32_t sum_x = 0;
	int32_t sum_y = 0;


	// calculate average
	for (int i = 0; i < idx; i++)
	{
		sum_x += x_values[i];
		sum_y += y_values[i];
	}

	float avg_x = (float)sum_x / (float)idx;
	float avg_y = (float)sum_y / (float)idx;


	int8_t min_x = round(avg_x) - 12;
	int8_t max_x = round(avg_x) + 12;

	int8_t min_y = round(avg_y) - 12;
	int8_t max_y = round(avg_y) + 12;

	int32_t sum_x2 = 0;
	int32_t sum_y2 = 0;
	int32_t count_2 = 0;

	// calculate average without outliars
	for (int i = 0; i < idx; i++)
	{
		int8_t x = x_values[i];
		int8_t y = y_values[i];
		if (x >= min_x && x <= max_x
			&& y >= min_y && x <= max_y)
		{
			sum_x2 += x;
			sum_y2 += y;
			count_2++;
		}

		//sum_x += x >= min_x && x <= max_x? x: 0;
		//sum_y += y >= min_y && x <= max_y? y : 0;
	}

	float avg_x2 = (float)sum_x2 / (float)count_2;
	float avg_y2 = (float)sum_y2 / (float)count_2;

//	if (sum_x != 0 || sum_y != 0) {
//		printf("optical flow: x %f %f, y %f %f\n", avg_x, avg_x2, avg_y, avg_y2);
//	}

	*out_x = avg_x2;
	*out_y = avg_y2;
}

void OpticalFlow_RaspberryPi::_run_optflow()
{
    GyroSample gyro_sample;
    Vector2f flow_rate;
    uint8_t qual;
    uint64_t frame_time;


    while(true) {
        /* ... */
    	_calculate_flow(&flow_rate.x, &flow_rate.y, &frame_time);


        /* read the integrated gyro data */
        _get_integrated_gyros(frame_time, gyro_sample);


        /* compute gyro data and video frames
         * get flow rate to send it to the opticalflow driver
         */
        qual = 100;

        /* fill data frame for upper layers */
        pthread_mutex_lock(&_mutex);
        _pixel_flow_x_integral += flow_rate.x /
                                  HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX;
        _pixel_flow_y_integral += flow_rate.y /
                                  HAL_FLOW_PX4_FOCAL_LENGTH_MILLIPX;
        _integration_timespan += frame_time -
        						 _last_frame_time;
        _gyro_x_integral       += (gyro_sample.gyro.x - _last_gyro_rate.x) *
                                  (frame_time - _last_frame_time) /
                                  (gyro_sample.time_us - _last_integration_time);
        _gyro_y_integral       += (gyro_sample.gyro.y - _last_gyro_rate.y) /
                                  (gyro_sample.time_us - _last_integration_time) *
								  (frame_time - _last_frame_time);
        _surface_quality = qual;
        _data_available = true;
        pthread_mutex_unlock(&_mutex);

        _last_integration_time = gyro_sample.time_us;
        _last_gyro_rate = gyro_sample.gyro;
        _last_frame_time = frame_time;
    }

}
#endif
