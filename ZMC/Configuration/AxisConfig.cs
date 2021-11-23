namespace APAS.MotionLib.ZMC.Configuration
{
	public class AxisConfig
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
		/// 应用于复位伺服驱动器的输出IO。
		/// </summary>
		public int ResetIo { get; set; } = -1;

		/// <summary>
		/// 回零参数
		/// </summary>
		public HomingConfig Home { get; set; } = new HomingConfig();
	}
}
