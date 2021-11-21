using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace APAS.MotionLib.ZMC
{
	public class CardParam
	{
		List<AxisParam> axisParams = new List<AxisParam>();

		public  List<AxisParam> AxisParams
		{
			get { return axisParams; }
			set { axisParams = value; }
		}
	}
	/// <summary>
	/// 值为-1时不进行设置
	/// </summary>
	public  class AxisParam
	{

		/// <summary>
		/// 轴索引
		/// </summary>
		public int AxisIndex { get; set; } = -1;
		/// <summary>
		/// 脉冲当量
		/// </summary>
		public float AxisUnits { get; set; } = -1;

		/// <summary>
		/// 轴类型
		/// </summary>
		public int AxisType { get; set; } = -1;

		/// <summary>
		/// 回零参数
		/// </summary>
		public AxisHomeParam AxisHomeParam { get; set; } = new AxisHomeParam();
	}

	public class AxisHomeParam
	{
		/// <summary>
		/// 回零模式，默认16，负方向脉冲+编码器，Z向信号回零
		/// </summary>
		public int HomeMode { get; set; } = 16;

		/// <summary>
		/// 原点sensor接入的输入口序号
		/// </summary>
		public int HomeSensorIoIndex { get; set; } = -1;
		/// <summary>
		/// 原点sensor信号是否反转，一般常开输入需要反转
		/// </summary>
		public bool HomeSensorInvert { get; set; } = false;
		/// <summary>
		/// 正限位sensor接入的输入口序号
		/// </summary>
		public int PositiveSensorIoIndex { get; set; } = -1;
		/// <summary>
		/// 正限位sensor信号是否反转，一般常开输入需要反转
		/// </summary>
		public bool PositiveSensorInvert { get; set; } = false;
		/// <summary>
		/// 负限位sensor接入的输入口序号
		/// </summary>
		public int NegativeSensorIoIndex { get; set; } = -1;
		/// <summary>
		/// 负限位sensor信号是否反转，一般常开输入需要反转
		/// </summary>
		public bool NegativeSensorInvert { get; set; } = false;

		/// <summary>
		/// Home时的加速度
		/// </summary>
		public float Acceleration { get; set; }

		/// <summary>
		/// Home时的减速度
		/// </summary>
		public float Deceleration { get; set; }
	}

}
