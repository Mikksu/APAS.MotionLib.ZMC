using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace APAS.MotionLib.ZMC.Tests
{
    [TestClass()]
	public class ZMC_406Tests
	{
		[TestMethod()]
		public void UnitTestTest()
		{
			var mc = new ZMC_406("192.168.0.11", 0, ",ZMC406Cfg.json");
			mc.UnitTest(1);

			/*var t1 = Task.Run(() =>
			{
				mc.UnitTest(0);
			});
			
			var t2 = Task.Run(() =>
			{
				mc.UnitTest(1);
			});

			var tList = new List<Task>() { t1, t2 };
			Task.WaitAll(tList.ToArray());*/
		}
	}
}