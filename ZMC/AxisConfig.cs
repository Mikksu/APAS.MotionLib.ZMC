using System.Collections.Generic;

namespace APAS.MotionLib.ZMC
{
	public class McParam
	{
		List<AxisParam> axisParams = new List<AxisParam>();

		public List<AxisParam> AxisParams
		{
			get => axisParams;
			set
			{
				axisParams = value;
			}
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
		public int Index { get; set; } = -1;
		/// <summary>
		/// 脉冲当量
		/// </summary>
		public float Units { get; set; } = -1;

		/// <summary>
		/// 轴类型
		/// </summary>
		public int Type { get; set; } = -1;

		/// <summary>
		/// 回零参数
		/// </summary>
		public AxisHomeParam HomeParam { get; set; } = new AxisHomeParam();
	}

	public class AxisHomeParam
	{
		/// <summary>
		/// 回零模式，默认16，负方向脉冲+编码器，Z向信号回零
		/// </summary>
		public int Mode { get; set; } = 16;

		/// <summary>
		/// 原点sensor接入的输入口序号
		/// </summary>
		public int OrgIo { get; set; } = -1;
		/// <summary>
		/// 原点sensor信号是否反转，一般常开输入需要反转
		/// </summary>
		public bool OrgIoInv { get; set; } = false;
		/// <summary>
		/// 正限位sensor接入的输入口序号
		/// </summary>
		public int PelIo { get; set; } = -1;
		/// <summary>
		/// 正限位sensor信号是否反转，一般常开输入需要反转
		/// </summary>
		public bool PelIoInv { get; set; } = false;
		/// <summary>
		/// 负限位sensor接入的输入口序号
		/// </summary>
		public int NelIo { get; set; } = -1;
		/// <summary>
		/// 负限位sensor信号是否反转，一般常开输入需要反转
		/// </summary>
		public bool NelIoInv { get; set; } = false;

		/// <summary>
		/// Home时的加速度
		/// </summary>
		public float Acc { get; set; }

		/// <summary>
		/// Home时的减速度
		/// </summary>
		public float Dec { get; set; }
	}

}
