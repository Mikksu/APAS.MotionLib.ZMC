using System;
using System.Collections.Generic;
using System.Threading;
using APAS__MotionControl;
using APAS__MotionControl.Core;
using log4net;
using cszmcaux;
using System.IO;
using Newtonsoft.Json;
using System.Runtime.CompilerServices;
using System.Linq;
using APAS.MotionLib.ZMC.Configuration;
using System.Text;

/*
 注意：

 1. 应用该模板时，请注意将命名空间更改为实际名称。
 2. 该类中的所有Childxxx()方法中，请勿阻塞式调用实际的运动控制器库函数，因为在APAS主程序中，可能会同时调用多个轴同步移动。
 3. 请保证所有的Childxxx()方法为线程安全。

*/

namespace APAS.MotionLib.ZMC
{
    public class ZMC_406 : MotionControllerBase
    {
        #region Variables

        private readonly string _configFileGts = "";
        private readonly string _configFileAxis = "ZMC406Cfg.json";

        private short _cardId = 0;
        private IntPtr _hMc;

        private McConfig _mcConfig;
        #endregion
        
        #region Constructors

        /// <summary>
        /// 注意：类名应为 “MotionController",请勿更改。
        /// </summary>
        /// <param name="portName"></param>
        /// <param name="baudRate"></param>
        /// <param name="logger"></param>

        public ZMC_406(string portName, int baudRate, string config, ILog logger = null) : base(portName, baudRate, logger)
        {
            var configs = config.Split(',');
            if (configs.Length == 2)
            {
                _configFileGts = configs[0];
                _configFileAxis = configs[1];
            }

            // 加载自定义配置
            string paramPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, _configFileAxis);
            ReadParamFile(paramPath, ref _mcConfig);

            //TODO 此处初始化控制器参数；如果下列参数动态读取，则可在ChildInit()函数中赋值。
            AxisCount = 6; // 最大轴数
            MaxAnalogInputChannels = _mcConfig.Ain.MaxChannel; // 最大模拟量输入通道数
            MaxAnalogOutputChannels = 0; // 最大模拟量输出通道数
            MaxDigitalInputChannels = 24; // 最大数字量输入通道数
            MaxDigitalOutputChannels = 12; // 最大数字量输出通道数
        }

        #endregion

        #region Overrided Methods

        /// <summary>
        /// 初始化指定轴卡。
        /// </summary>
        protected override void ChildInit()
        {
            //TODO 1.初始化运动控制器对象，例如凌华轴卡、固高轴卡等。
            // 例如：初始化固高轴卡：gts.mc.GT_Open(portName, 1);

            //TODO 2.读取控制器固件版本，并赋值到属性 FwVersion

            //TODO 3.读取每个轴的信息，构建 InnerAxisInfoCollection，包括轴号和固件版本号。
            // 注意：InnerAxisInfoCollection 已在基类的构造函数中初始化
            // 例如： InnerAxisInfoCollection.Add(new AxisInfo(1, new Version(1, 0, 0)));

            //TODO 4.需要完成函数 ChildUpdateStatus()，否则会报NotImplementException异常。

            int rtn = zmcaux.ZAux_SearchEth(PortName, 100);

            CommandRtnCheck(rtn, "ZAux_SearchEth");


            rtn = zmcaux.ZAux_OpenEth(PortName, out _hMc);
            CommandRtnCheck(rtn, "ZAux_OpenEth");

            ApplyConfig(_hMc, _mcConfig);

        }

        /// <summary>
        /// 设置指定轴的加速度。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="acc">加速度值</param>
        protected override void ChildSetAcceleration(int axis, double acc)
        {

            int rtn = zmcaux.ZAux_Direct_SetAccel(_hMc, axis, (float)acc);
            CommandRtnCheck(rtn, "ZAux_Direct_SetAccel  in ChildSetAcceleration");
        }

        /// <summary>
        /// 设置指定轴的减速度。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="dec">减速度值</param>
        protected override void ChildSetDeceleration(int axis, double dec)
        {
            int rtn = zmcaux.ZAux_Direct_SetDecel(_hMc, axis, (float)dec);
            CommandRtnCheck(rtn, "ZAux_Direct_SetDecel  in ChildSetAcceleration");
        }

        /// <summary>
        /// 指定轴回机械零点。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="hiSpeed">快速找机械原点的速度值。如不适用请忽略。</param>
        /// <param name="creepSpeed">找到机械原点后返回零位的爬行速度。如不适用请忽略。</param>
        protected override void ChildHome(int axis, double hiSpeed, double creepSpeed)
        {
            /*
             * 耗时操作。当执行操作时，请轮询轴状态，并调用 RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, xxx)); 
             * 以实时刷新UI上的位置。       
             */
            int rtn;
            int axisMoveStatus = 0;
            var homeParam = FindAxisConfig(axis, ref _mcConfig).Home;

            CheckAxisIdle(axis);

            SetAcceleration(axis, homeParam.Acc);
            SetDeceleration(axis, homeParam.Dec);

            rtn = zmcaux.ZAux_Direct_SetCreep(_hMc, axis, (float)creepSpeed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetCreep ");

            rtn = zmcaux.ZAux_Direct_SetSpeed(_hMc, axis, (float)hiSpeed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetSpeed ");

            rtn = zmcaux.ZAux_Direct_Single_Datum(_hMc, axis, homeParam.Mode);
            CommandRtnCheck(rtn, nameof(zmcaux.ZAux_Direct_Single_Datum));

            Thread.Sleep(100);
            float position=0;
            do
            {
                rtn=zmcaux.ZAux_Direct_GetIfIdle(_hMc, axis, ref axisMoveStatus);
                CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");

                rtn = zmcaux.ZAux_Direct_GetMpos(_hMc, axis, ref position);
                CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

                RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)position));

                Thread.Sleep(50);
            } while (axisMoveStatus == 0);
            Thread.Sleep(100);
            AxisStatueCheck(_hMc, axis);

            // 清空位置
            rtn = zmcaux.ZAux_Direct_SetMpos(_hMc, axis, 0);
            CommandRtnCheck(rtn, nameof(zmcaux.ZAux_Direct_SetMpos));

            rtn = zmcaux.ZAux_Direct_SetDpos(_hMc, axis, 0);
            CommandRtnCheck(rtn, nameof(zmcaux.ZAux_Direct_SetDpos));

            // 将该轴标记为已Home
            rtn = zmcaux.ZAux_Modbus_Set0x(_hMc, 0, 1, new byte[] { 1 });
            CommandRtnCheck(rtn, nameof(zmcaux.ZAux_Modbus_Set0x));

            RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, 0, true));

        }

        /// <summary>
        /// 移动指定轴（相对移动模式）。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="speed">移动速度。该速度根据APAS主程序的配置文件计算得到。计算方法为MaxSpeed * 速度百分比。</param>
        /// <param name="distance">相对移动的距离。该距离已被APAS主程序转换为轴卡对应的实际单位。例如对于脉冲方式，
        /// 该值已转换为步数；对于伺服系统，该值已转换为实际距离。</param>
        /// <param name="fastMoveRequested">是否启用快速移动模式。如不适用请忽略。</param>
        /// <param name="microstepRate">当启用快速移动模式时的驱动器细分比值。如不适用请忽略。</param>
        protected override void ChildMove(int axis, double speed, double distance,
            bool fastMoveRequested = false, double microstepRate = 0)
        {
            /*
             * 耗时操作。当执行操作时，请轮询轴状态，并调用 RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, xxx)); 
             * 以实时刷新UI上的位置。       
            */
            int axisMoveStatus = 0;

            CheckAxisIdle(axis);

            var rtn= zmcaux.ZAux_Direct_SetSpeed(_hMc, axis, (float)speed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetSpeed ");

            rtn = zmcaux.ZAux_Direct_Single_Move(_hMc, axis, (float)distance);

            CommandRtnCheck(rtn, "ZAux_Direct_Single_Move ");

            Thread.Sleep(100);
            float position = 0;

            do
            {
                rtn= zmcaux.ZAux_Direct_GetIfIdle(_hMc, axis, ref axisMoveStatus);
                CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");

                rtn = zmcaux.ZAux_Direct_GetMpos(_hMc, axis, ref position);
                CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

                RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)position));
                Thread.Sleep(50);
            } while (axisMoveStatus == 0);
            Thread.Sleep(100);

            AxisStatueCheck(_hMc, 0);

            rtn = zmcaux.ZAux_Direct_GetMpos(_hMc, axis, ref position);
            CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

            RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)position));
        }


        /// <summary>
        /// 移动指定轴到绝对位置（绝对移动模式）。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="speed">移动速度</param>
        /// <param name="position">绝对目标位置</param>
        /// <param name="fastMoveRequested">是否启用快速移动模式。如不适用请忽略。</param>
        /// <param name="microstepRate">当启用快速移动模式时的驱动器细分比值。如不适用请忽略。</param>
        protected override void ChildMoveAbs(int axis, double speed, double position, bool fastMoveRequested = false,
            double microstepRate = 0)
        {
            int axisMoveStatus = 0;

            CheckAxisIdle(axis);

            var rtn = zmcaux.ZAux_Direct_SetSpeed(_hMc, axis, (float)speed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetSpeed ");

            rtn = zmcaux.ZAux_Direct_Single_MoveAbs(_hMc, axis, (float)position);

            CommandRtnCheck(rtn, "ZAux_Direct_Single_Move ");

            Thread.Sleep(100);
            float pos = 0;

            do
            {
                rtn = zmcaux.ZAux_Direct_GetIfIdle(_hMc, axis, ref axisMoveStatus);
                CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");
                rtn = zmcaux.ZAux_Direct_GetMpos(_hMc, axis, ref pos);
                CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

                RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)pos));
                Thread.Sleep(50);
            } while (axisMoveStatus == 0);
            Thread.Sleep(100);

            AxisStatueCheck(_hMc, 0);
            rtn = zmcaux.ZAux_Direct_GetMpos(_hMc, axis, ref pos);
            CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

            RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)pos));
        }

        /// <summary>
        /// 开启励磁。
        /// </summary>
        /// <param name="axis">轴号</param>
        protected override void ChildServoOn(int axis)
        {
           int rtn= zmcaux.ZAux_Direct_SetOp(_hMc, 12 + axis, 1);
            CommandRtnCheck(rtn, "ZAux_Direct_SetOp ");

        }

        /// <summary>
        /// 关闭励磁。
        /// </summary>
        /// <param name="axis">轴号</param>
        protected override void ChildServoOff(int axis)
        {
            CheckAxisIdle(axis);

            int rtn = zmcaux.ZAux_Direct_SetOp(_hMc, 12 + axis, 0);
            CommandRtnCheck(rtn, "ZAux_Direct_SetOp ");
        }

        /// <summary>
        /// 读取最新的绝对位置。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <returns>最新绝对位置</returns>
        protected override double ChildUpdateAbsPosition(int axis)
        {
            //pClock 读取控制器时钟，默认值为：NULL，即不用读取控制器时钟
            //count  读取的轴数，默认为 1。正整数。
            float pos = 0;
            int rtn = zmcaux.ZAux_Direct_GetMpos(_hMc, axis, ref pos);
            CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");
            return pos;
        }

        /// <summary>
        /// 更新指定轴状态。
        /// <para>注意：请在该函数调用RaiseAxisStateUpdatedEvent()函数，以通知APAS主程序当前轴的状态已更新。</para>
        /// </summary>
        /// <param name="axis">轴号</param>
        protected override void ChildUpdateStatus(int axis)
        {
            // 注意:
            // 1. 读取完状态后请调用 RaiseAxisStateUpdatedEvent 函数。
            // 2. 实例化 AxisStatusArgs 时请传递所有参数。
            // RaiseAxisStateUpdatedEvent(new AxisStatusArgs(int.MinValue, double.NaN, false, false));

        }

        /// <summary>
        /// 更新所有轴状态。
        /// <see cref="ChildUpdateStatus(int)"/>
        /// </summary>
        protected override void ChildUpdateStatus()
        {
            // 注意:
            // 1. 读取完状态后请循环调用 RaiseAxisStateUpdatedEvent 函数，
            //    例如对于 8 轴轴卡，请调用针对8个轴调用 8 次 RaiseAxisStateUpdatedEvent 函数。
            // 2. 实例化 AxisStatusArgs 时请传递所有参数。
            //// RaiseAxisStateUpdatedEvent(new AxisStatusArgs(int.MinValue, double.NaN, false, false));


        }


        /// <summary>
        /// 清除指定轴的错误。
        /// </summary>
        /// <param name="axis">轴号</param>
        protected override void ChildResetFault(int axis)
        {
            var axisConf = FindAxisConfig(axis, ref _mcConfig);

            if (axisConf.ResetIo < 0)
                return;

            // 复位伺服驱动器
            zmcaux.ZAux_Direct_SetOp(_hMc, axisConf.ResetIo, 1);
            Thread.Sleep(100);
            zmcaux.ZAux_Direct_SetOp(_hMc, axisConf.ResetIo, 0);
        }


        #region IO Controller

        /// <summary>
        /// 设置指定数字输出端口的状态。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <param name="isOn">是否设置为有效电平</param>
        protected override void ChildSetDigitalOutput(int port, bool isOn)
        {
                 
            int  rtn;
            if (isOn)
                rtn = zmcaux.ZAux_Direct_SetOp(_hMc, port, 1);
            else
                rtn = zmcaux.ZAux_Direct_SetOp(_hMc, port, 0);

            CommandRtnCheck(rtn, "ZAux_Direct_SetOp");

        }

        /// <summary>
        /// 读取指定数字输出端口。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <returns>端口状态。True表示端口输出为有效电平。</returns>
        protected override bool ChildReadDigitalOutput(int port)
        {
            uint pivalue = 0;
            int rtn = zmcaux.ZAux_Direct_GetOp(_hMc, port, ref pivalue);
            CommandRtnCheck(rtn, "ZAux_Direct_GetOp");
            return pivalue != 0;

        }

        /// <summary>
        /// 读取所有数字输出端口。
        /// </summary>
        /// <returns>端口状态列表。True表示端口输出为有效电平。</returns>
        protected override IReadOnlyList<bool> ChildReadDigitalOutput()
        {
            uint[] outputStatus = new uint[1];
           int rtn= zmcaux.ZAux_Direct_GetOutMulti(_hMc, 0, 15, outputStatus);
            CommandRtnCheck(rtn, "ZAux_Direct_GetOutMulti");
            bool[] states = new bool[16];
            for(int i=0;i<16;i++)
            {
                states[i] = (outputStatus[0] & (1 << i)) != 0;
            }
            return states;
        }

        /// <summary>
        /// 读取指定数字输入端口。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <returns>端口状态。True表示端口输出为有效电平。</returns>
        protected override bool ChildReadDigitalInput(int port)
        {
            uint pivalue = 0;
            int rtn = zmcaux.ZAux_Direct_GetIn(_hMc, port, ref pivalue);
            CommandRtnCheck(rtn, "ZAux_Direct_GetIn");
            return pivalue != 0;

        }

        /// <summary>
        /// 读取所有数字输入端口。
        /// </summary>
        /// <returns>端口状态列表。True表示端口输出为有效电平。</returns>
        protected override IReadOnlyList<bool> ChildReadDigitalInput()
        {
            int[] outputStatus = new int[1];
            int rtn = zmcaux.ZAux_Direct_GetInMulti(_hMc, 0, 15, outputStatus);
            CommandRtnCheck(rtn, "ZAux_Direct_GetInMulti");
            bool[] states = new bool[16];
            for (int i = 0; i < 16; i++)
            {
                states[i] = (outputStatus[0] & (1 << i)) != 0;
            }
            return states;
        }


        #endregion

        #region Analog Controller

        /// <summary>
        /// 读取所有模拟输入端口的电压值。
        /// </summary>
        /// <returns>电压值列表。</returns>
        protected override IReadOnlyList<double> ChildReadAnalogInput()
        {
            List<double> values = new List<double>();

			for (int i = 0; i < MaxAnalogInputChannels; i++)
			{
                float val = 0.0f;
                zmcaux.ZAux_Direct_GetAD(_hMc, _mcConfig.Ain.IndexStart + i, ref val);
                values.Add((double)val);
            }

            return values;
        }

        /// <summary>
        /// 读取指定模拟输入端口的电压值。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <returns></returns>
        protected override double ChildReadAnalogInput(int port)
        {
            float val = 0.0f;
            zmcaux.ZAux_Direct_GetAD(_hMc, _mcConfig.Ain.IndexStart + port, ref val);
            return (double)val;
        }

        /// <summary>
        /// 读取所有模拟输出端口的电压值。
        /// </summary>
        /// <returns>电压值列表。</returns>
        protected override IReadOnlyList<double> ChildReadAnalogOutput()
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// 读取指定模拟输出端口的电压值。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <returns></returns>
        protected override double ChildReadAnalogOutput(int port)
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// 设置指定模拟输出端口的电压值。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <param name="value">电压值</param>
        protected override void ChildSetAnalogOutput(int port, double value)
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// 打开指定模拟输出端口的输出。
        /// </summary>
        /// <param name="port">端口号</param>
        protected override void ChildAnalogOutputOn(int port)
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// 关闭指定模拟输出端口的输出。
        /// </summary>
        /// <param name="port">端口号</param>
        protected override void ChildAnalogOutputOff(int port)
        {
            throw new NotSupportedException();
        }

        #endregion

        /// <summary>
        /// 在指定轴上执行自动接触检测功能。
        /// <para>该功能适用于Irixi M12控制器。</para>
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="analogInputPort">模拟输入端口号</param>
        /// <param name="vth">阈值电压</param>
        /// <param name="distance">最大移动距离</param>
        /// <param name="speed">移动速度</param>
        protected override void ChildAutoTouch(int axis, int analogInputPort, double vth, double distance, double speed)
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// 执行快速线性扫描。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="range">扫描范围</param>
        /// <param name="interval">反馈信号采样间隔</param>
        /// <param name="speed">移动速度</param>
        /// <param name="analogCapture">反馈信号捕获端口</param>
        /// <param name="scanResult">扫描结果列表（X:位置，Y:反馈信号）</param>
        protected override void ChildStartFast1D(int axis, double range, double interval, double speed,
            int analogCapture, out IEnumerable<Point2D> scanResult)
        {
            var point2Ds = new List<Point2D>();

            StringBuilder respon = new StringBuilder();

            // 总采样点数，注意此值必须为2的倍数
            int totalSamplingPoints = _mcConfig.Scope.Deepth * 2;
            if (totalSamplingPoints < 2000)
                totalSamplingPoints = 2000;

            // 采样间隔，单位ms
            // 注意：此值和上述值共同决定了能够采样的最大时间，例如1000(点) * 5ms = 5000ms
            int samplingIntervalMs = _mcConfig.Scope.InvtervalMs;
            if (samplingIntervalMs < 1)
                samplingIntervalMs = 1;

            var command = $"SCOPE(ON,{samplingIntervalMs},0,{totalSamplingPoints},MPOS({axis}),AIN({analogCapture + _mcConfig.Ain.IndexStart}))";
            zmcaux.ZAux_Execute(_hMc, command, respon, 100);
            zmcaux.ZAux_Trigger(_hMc);
            Move(axis, speed, range);
            zmcaux.ZAux_Execute(_hMc, $"SCOPE_POS(OFF)", respon, 100);
            zmcaux.ZAux_Execute(_hMc, $"?SCOPE_POS", respon, 100);

            if (!int.TryParse(respon.ToString(), out var pCnt))
                throw new Exception("无法获取采样的数据点总数。");

            var pBuf = new float[totalSamplingPoints];
            zmcaux.ZAux_Direct_GetTable(_hMc, 0, totalSamplingPoints, pBuf);

            // AIN采样值所在的位置
            int startAin = totalSamplingPoints / 2;
            for (int i = 0; i < pCnt; i++)
			{
                point2Ds.Add(new Point2D(pBuf[i], pBuf[i + startAin]));
            }

            scanResult = point2Ds;
        }

        /// <summary>
        /// 执行双通道快速线性扫描。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="range">扫描范围</param>
        /// <param name="interval">第1路反馈信号采样间隔</param>
        /// <param name="speed">移动速度</param>
        /// <param name="analogCapture">反馈信号捕获端口</param>
        /// <param name="scanResult">第1路扫描结果列表（X:位置，Y:反馈信号）</param>
        /// <param name="analogCapture2">第2路反馈信号采样间隔</param>
        /// <param name="scanResult2">第2路扫描结果列表（X:位置，Y:反馈信号）</param>
        protected override void ChildStartFast1D(int axis, double range, double interval, double speed,
            int analogCapture,
            out IEnumerable<Point2D> scanResult, int analogCapture2, out IEnumerable<Point2D> scanResult2)
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// 执行快速盲扫。
        /// </summary>
        /// <param name="hAxis">水平轴轴号</param>
        /// <param name="vAxis">垂直轴轴号</param>
        /// <param name="range">扫描区域（正方形）的边长</param>
        /// <param name="gap">扫描螺旋线路的间隔</param>
        /// <param name="interval">每条扫描线上反馈信号采样间隔</param>
        /// <param name="hSpeed">水平轴扫描速度</param>
        /// <param name="vSpeed">垂直轴扫描速度</param>
        /// <param name="analogCapture">反馈信号捕获端口</param>
        /// <param name="scanResult">扫描结果列表（X:水平轴坐标，Y:垂直轴坐标，Z:反馈信号）</param>
        protected override void ChildStartBlindSearch(int hAxis, int vAxis, double range, double gap,
            double interval, double hSpeed, double vSpeed, int analogCapture, out IEnumerable<Point3D> scanResult)
        {
            throw new NotSupportedException();
        }

        /// <summary>
        /// 停止所有轴移动。
        /// </summary>
        protected override void ChildStop()
        {
            int rtn;
            for (int i = 0; i < AxisCount; i++)
            {
                rtn = zmcaux.ZAux_Direct_Single_Cancel(_hMc, i, 0);
                //CommandRtnCheck(rtn, "ZAux_Direct_Single_Cancel");
            }

        }

		protected override void ChildEmergencyStop()
		{
            int rtn;
            for (int i = 0; i < AxisCount; i++)
            {
                rtn = zmcaux.ZAux_Direct_Single_Cancel(_hMc, i, 3);
                //CommandRtnCheck(rtn, "ZAux_Direct_Single_Cancel");
            }
        }

		/// <summary>
		/// 关闭运动控制器，并销毁运动控制器实例。
		/// </summary>
		protected override void ChildDispose()
        {
          int rtn=  zmcaux.ZAux_Close(_hMc);
            CommandRtnCheck(rtn, "ZAux_Close");

        }

        /// <summary>
        /// 检查移动速度。
        /// <para>如无需检查，请保持该函数为空。</para>
        /// </summary>
        /// <param name="speed">速度</param>
        protected override void CheckSpeed(double speed)
        {

        }

        /// <summary>
        /// 检查控制器状态。
        /// </summary>
        protected override void CheckController()
        {
            base.CheckController(); // 请勿删除该行。

        }


		#endregion

		private void CommandRtnCheck(int rtnValue, [CallerMemberName] string funcName = "")
        {
            string errorInfo = string.Empty;
            switch (rtnValue)
            {
                case 0:
                    return;
                case 217:
                    errorInfo = "控制器不支持或禁止的功能";
                    break;
                case 218:
                    errorInfo = "调用传递的参数错误";
                    break;
                case 272:
                    errorInfo = "子卡不存在";
                    break;
                case 282:
                    errorInfo = "不支持的功能";
                    break;
                case 1008:
                    errorInfo = "运动模块输入参数错误";
                    break;
                case 1009:
                    errorInfo = "运动中，无法操作";
                    break;
                case 1010:
                    errorInfo = "暂停等重复操作";
                    break;
                case 1012:
                    errorInfo = "当前运动不支持暂停";
                    break;
                case 1014:
                    errorInfo = "ATYPE不支持";
                    break;
                case 1015:
                    errorInfo = "ZCAN的ATYPE冲突";
                    break;
                case 2023:
                    errorInfo = "试图修改只读状态参数";
                    break;
                case 2024:
                    errorInfo = "数组越界";
                    break;
                case 2025:
                    errorInfo = "变量数操过控制器规格";
                    break;

                case 2026:
                    errorInfo = "数组数操过控制器规格";
                    break;
                case 2027:
                    errorInfo = "数组空间操过控制器规格";
                    break;
                case 6025:
                    errorInfo = "轴数超过";
                    break;
                case 20007:
                    errorInfo = "串口打开失败";
                    break;
                case 20008:
                    errorInfo = "网络打开失败";
                    break;
                default:
                    errorInfo = "请查看指令返回值详细说明";
                    break;
            }
            if (string.IsNullOrEmpty(errorInfo))
            {
                return;
            }
            throw new Exception($"{funcName} 功能异常，错误码： {rtnValue}, 异常信息：{errorInfo}");
        }

        private void AxisStatueCheck(IntPtr cardHandle, int axisIndex)
        {
            int axiaStatus = 0;
            int rtn = zmcaux.ZAux_Direct_GetAxisStatus(_hMc, 0, ref axiaStatus);
            CommandRtnCheck(rtn, "ZAux_Direct_GetAxisStatus");
            string statueErrorInfo = string.Empty;
            switch (axiaStatus)
            {
                case 0:
                    break;
                case 0x2:
                    statueErrorInfo = "随动误差超限报警";
                    break;
                case 0x4:
                    statueErrorInfo = "与远程轴通讯错误";
                    break;
                case 0x8:
                    statueErrorInfo = "远程驱动器报错";
                    break;
                case 0x10:
                    statueErrorInfo = "正向硬限位";
                    break;
                case 0x20:
                    statueErrorInfo = "反向硬限位";
                    break;
                case 0x80:
                    statueErrorInfo = "随动误差超限报警";
                    break;
                case 0x40:
                    statueErrorInfo = "回原点中";
                    break;
                case 0x100:
                    statueErrorInfo = "随动误差超限出错";
                    break;
                case 0x200:
                    statueErrorInfo = "超过正向软限位";
                    break;
                case 0x400:
                    statueErrorInfo = "超过负向软限位";
                    break;
                case 0x800:
                    statueErrorInfo = "CANCLE执行中";
                    break;
                case 0x1000:
                    statueErrorInfo = "脉冲频率操过MAX_SPEED限制";
                    break;

                case 0x4000:
                    statueErrorInfo = "机械手指令坐标错误";
                    break;
                case 0x40000:
                    statueErrorInfo = "电源异常";
                    break;
                case 0x200000:
                    statueErrorInfo = "运动中触发特殊运动指令失败";
                    break;
                case 0x400000:
                    statueErrorInfo = "报警信号输入";
                    break;
                case 0x800000:
                    statueErrorInfo = "轴进入暂停状态";
                    break;

            }
            if (string.IsNullOrEmpty(statueErrorInfo))
                return;
            throw new Exception($"{axisIndex} 号轴状态异常，{statueErrorInfo}");
        }

        private void ReadParamFile(string filePath, ref McConfig cardParam)
		{
            if(!File.Exists(filePath))
			{
                return;
			}
            string jsonInfo=   File.ReadAllText(filePath);
            try
			{
                cardParam = JsonConvert.DeserializeObject<McConfig>(jsonInfo);
            }
            catch (FormatException ex)
			{
                throw new Exception($"{filePath} ,配置文件加载异常");
			}

		}

        /// <summary>
        /// 应用配置文件对轴卡进行配置
        /// </summary>
        /// <param name="cardHandle"></param>
        /// <param name="cardParam"></param>
        private void ApplyConfig(IntPtr cardHandle, McConfig cardParam)
		{
            int rtn;
            foreach (var mem in cardParam.Axes)
            {
                if (mem.Index < 0)
                {
                    break;
                }
                if (mem.Type > 0)
                {
                    rtn = zmcaux.ZAux_Direct_SetAtype(cardHandle, mem.Index, mem.Type);
                    CommandRtnCheck(rtn, "ZAux_Direct_SetAtype in LoadParam Function");
                }

                if (mem.Units > 0)
                {
                    rtn = zmcaux.ZAux_Direct_SetUnits(cardHandle, mem.Index, mem.Units);
                    CommandRtnCheck(rtn, "ZAux_Direct_SetUnits in LoadParam Function");
                }

                if (mem.Home.OrgIo >= -1)
                {
                    rtn = zmcaux.ZAux_Direct_SetDatumIn(cardHandle, mem.Index, mem.Home.OrgIo);
                    CommandRtnCheck(rtn, nameof(zmcaux.ZAux_Direct_SetDatumIn));
                }
                if (mem.Home.OrgIoInv)
                {
                    rtn = zmcaux.ZAux_Direct_SetInvertIn(cardHandle, mem.Home.OrgIo, 1);
                }

                if (mem.Home.PelIo >= -1)
                {
                    rtn = zmcaux.ZAux_Direct_SetFwdIn(cardHandle, mem.Index, mem.Home.PelIo);
                    CommandRtnCheck(rtn, "ZAux_Direct_SetFwdIn in LoadParam Function");
                }
                if (mem.Home.PelIoInv)
                {
                    rtn = zmcaux.ZAux_Direct_SetInvertIn(cardHandle, mem.Home.PelIo, 1);
                }

                if (mem.Home.NelIo >= -1)
                {
                    rtn = zmcaux.ZAux_Direct_SetRevIn(cardHandle, mem.Index, mem.Home.NelIo);
                    CommandRtnCheck(rtn, "ZAux_Direct_SetFwdIn in LoadParam Function");
                }
                if (mem.Home.NelIoInv)
                {
                    rtn = zmcaux.ZAux_Direct_SetInvertIn(cardHandle, mem.Home.NelIo, 1);
                }
            }
		}

        /// <summary>
        /// 检查轴是否空闲
        /// </summary>
        /// <param name="axis"></param>
        private void CheckAxisIdle(int axis)
		{
            int axisMoveStatus = 0;
            var rtn = zmcaux.ZAux_Direct_GetIfIdle(_hMc, axis, ref axisMoveStatus);
            CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");

            if (axisMoveStatus == 0)
            {
                throw new Exception($"轴号 {axis},运行中");
            }
        }


        private AxisConfig FindAxisConfig(int axis, ref McConfig cardParam)
		{
            var param = cardParam.Axes.FirstOrDefault(x => x.Index == axis);
            if (param == null)
                throw new NullReferenceException($"无法在文件{_configFileAxis}中找到轴{axis}的配置参数。");

            return param;
		}


        public void UnitTest()
		{
            Init();
            ResetFault(0);
            ServoOn(0);
            Home(0, 100000, 10000);
            SetAcceleration(0, 1000000);
            SetDeceleration(0, 5000000);
            Move(0, 100000, 200000);

            StartFast1D(0, 10000, 10, 100000, 0, out var points);

            ServoOff(0);

			for (int i = 0; i < 10; i++)
			{
                var val = ReadAnalogInput();
            }
		}
    }
}
