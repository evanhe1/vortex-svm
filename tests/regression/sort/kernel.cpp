#include <stdint.h>
#include <vx_intrinsics.h>
#include <vx_spawn.h>
#include "common.h"

void kernel_body(int task_id, kernel_arg_t *__UNIFORM__ arg)
{
	uint32_t num_points = arg->num_points;
	auto src_ptr = (TYPE *)0x2040; //(TYPE*)(arg->src_addr) - 0xf0000000;
	auto dst_ptr = (TYPE *)0x20c0; //(TYPE*)(arg->dst_addr)- 0xf0000000;

	auto ref_value = src_ptr[task_id];

	uint32_t pos = 0;
	for (uint32_t i = 0; i < num_points; ++i)
	{
		auto cur_value = src_ptr[i];
		pos += (cur_value < ref_value) || ((cur_value == ref_value) && (i < task_id));
	}
	dst_ptr[pos] = ref_value;
}

int main()
{
	kernel_arg_t *arg = (kernel_arg_t *)csr_read(VX_CSR_MSCRATCH);
	vx_spawn_tasks(arg->num_points, (vx_spawn_tasks_cb)kernel_body, arg);
	return 0;
}
