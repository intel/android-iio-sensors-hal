#ifndef FILTERING_H
#define FILTERING_H

#define SAMPLE_SIZE 7

struct circ_buff
{
	float* buff;
	unsigned int idx;
	unsigned int count;
	unsigned int size;
};

struct filter
{
	struct circ_buff* x_buff;
	struct circ_buff* y_buff;
	struct circ_buff* z_buff;
};

void add_to_buff(struct circ_buff* circ_buff, float val);
float median(float* queue, unsigned int size);
void denoise_median(struct sensors_event_t* event, struct sensor_info_t* info);

#endif
