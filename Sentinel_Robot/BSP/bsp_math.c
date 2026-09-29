#include "bsp_math.h"

/**
  * @brief  转角限制到±PI
  * @param  输入转角
  * @retval 输出转角
  */
fp32 limit_pi(fp32 in)
{
	while(in < -PI || in > PI)
	{
		if (in < -PI)
			in = in + PI + PI;
		if (in > PI)
			in = in - PI - PI;
	}
	return in;
}