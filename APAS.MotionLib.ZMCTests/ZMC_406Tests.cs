using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
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
			mc.UnitTest();

		}
	}
}