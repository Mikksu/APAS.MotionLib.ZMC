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

        short m_cardId = 0;
        IntPtr m_cardHandle ;

        McParam m_cardParam;
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

            m_cardParam = new McParam();

            var configs = config.Split(',');
            if (configs.Length == 2)
            {
                _configFileGts = configs[0];
                _configFileAxis = configs[1];
            }

            //TODO 此处初始化控制器参数；如果下列参数动态读取，则可在ChildInit()函数中赋值。
            AxisCount = 6; // 最大轴数
            MaxAnalogInputChannels = 8; // 最大模拟量输入通道数
            MaxAnalogOutputChannels = 0; // 最大模拟量输出通道数
            MaxDigitalInputChannels = 8; // 最大数字量输入通道数
            MaxDigitalOutputChannels = 8; // 最大数字量输出通道数


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


            rtn = zmcaux.ZAux_OpenEth(PortName, out m_cardHandle);
            CommandRtnCheck(rtn, "ZAux_OpenEth");

            string paramPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, _configFileAxis);
            ReadParamFile(paramPath, ref m_cardParam);

            LoadParam(m_cardHandle, m_cardParam);

        }

        /// <summary>
        /// 设置指定轴的加速度。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="acc">加速度值</param>
        protected override void ChildSetAcceleration(int axis, double acc)
        {

            int rtn = zmcaux.ZAux_Direct_SetAccel(m_cardHandle, axis, (float)acc);
            CommandRtnCheck(rtn, "ZAux_Direct_SetAccel  in ChildSetAcceleration");


        }

        /// <summary>
        /// 设置指定轴的减速度。
        /// </summary>
        /// <param name="axis">轴号</param>
        /// <param name="dec">减速度值</param>
        protected override void ChildSetDeceleration(int axis, double dec)
        {
            int rtn = zmcaux.ZAux_Direct_SetDecel(m_cardHandle, axis, (float)dec);
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
            var homeParam = FindAxisParam(axis, ref m_cardParam).HomeParam;

            rtn = zmcaux.ZAux_Direct_GetIfIdle(m_cardHandle, axis, ref axisMoveStatus);
            CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");

            if (axisMoveStatus == 0)
                throw new Exception($"轴号 {axis},运行中");

            SetAcceleration(axis, homeParam.Acc);
            SetDeceleration(axis, homeParam.Dec);

            rtn = zmcaux.ZAux_Direct_SetCreep(m_cardHandle, axis, (float)creepSpeed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetCreep ");

            rtn = zmcaux.ZAux_Direct_SetSpeed(m_cardHandle, axis, (float)hiSpeed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetSpeed ");

            rtn = zmcaux.ZAux_Direct_Single_Datum(m_cardHandle, axis, homeParam.Mode);
            CommandRtnCheck(rtn, nameof(zmcaux.ZAux_Direct_Single_Datum));

            Thread.Sleep(100);
            float position=0;
            do
            {
                rtn=zmcaux.ZAux_Direct_GetIfIdle(m_cardHandle, axis, ref axisMoveStatus);
                CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");

                rtn = zmcaux.ZAux_Direct_GetMpos(m_cardHandle, axis, ref position);
                CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

                RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)position));

                Thread.Sleep(50);
            } while (axisMoveStatus == 0);
            Thread.Sleep(100);
            AxisStatueCheck(m_cardHandle, axis);
            rtn = zmcaux.ZAux_Direct_GetMpos(m_cardHandle, axis, ref position);
            CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");
            RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)position,true));

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
            int rtn;
            rtn= zmcaux.ZAux_Direct_GetIfIdle(m_cardHandle, axis, ref axisMoveStatus);
            CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");

            if (axisMoveStatus == 0)
            {
                throw new Exception($"轴号 {axis},运行中");
            }

            rtn= zmcaux.ZAux_Direct_SetSpeed(m_cardHandle, axis, (float)speed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetSpeed ");

            rtn = zmcaux.ZAux_Direct_Single_Move(m_cardHandle, axis, (float)distance);

            CommandRtnCheck(rtn, "ZAux_Direct_Single_Move ");

            Thread.Sleep(100);
            float position = 0;

            do
            {
                rtn= zmcaux.ZAux_Direct_GetIfIdle(m_cardHandle, axis, ref axisMoveStatus);
                CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");
                rtn = zmcaux.ZAux_Direct_GetMpos(m_cardHandle, axis, ref position);
                CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

                RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)position));
                Thread.Sleep(50);
            } while (axisMoveStatus == 0);
            Thread.Sleep(100);

            AxisStatueCheck(m_cardHandle, 0);

            rtn = zmcaux.ZAux_Direct_GetMpos(m_cardHandle, axis, ref position);
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
            int rtn;
            rtn = zmcaux.ZAux_Direct_GetIfIdle(m_cardHandle, axis, ref axisMoveStatus);
            CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");

            if (axisMoveStatus == 0)
            {
                throw new Exception($"轴号 {axis},运行中");
            }
            rtn = zmcaux.ZAux_Direct_SetSpeed(m_cardHandle, axis, (float)speed);
            CommandRtnCheck(rtn, "ZAux_Direct_SetSpeed ");



            rtn = zmcaux.ZAux_Direct_Single_MoveAbs(m_cardHandle, axis, (float)position);

            CommandRtnCheck(rtn, "ZAux_Direct_Single_Move ");

            Thread.Sleep(100);
            float pos = 0;

            do
            {
                rtn = zmcaux.ZAux_Direct_GetIfIdle(m_cardHandle, axis, ref axisMoveStatus);
                CommandRtnCheck(rtn, "ZAux_Direct_GetIfIdle ");
                rtn = zmcaux.ZAux_Direct_GetMpos(m_cardHandle, axis, ref pos);
                CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

                RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)pos));
                Thread.Sleep(50);
            } while (axisMoveStatus == 0);
            Thread.Sleep(100);

            AxisStatueCheck(m_cardHandle, 0);
            rtn = zmcaux.ZAux_Direct_GetMpos(m_cardHandle, axis, ref pos);
            CommandRtnCheck(rtn, "ZAux_Direct_GetMpos ");

            RaiseAxisStateUpdatedEvent(new AxisStatusArgs(axis, (double)pos));
        }

        /// <summary>
        /// 开启励磁。
        /// </summary>
        /// <param name="axis">轴号</param>
        protected override void ChildServoOn(int axis)
        {
           int rtn= zmcaux.ZAux_Direct_SetOp(m_cardHandle, 12 + axis, 1);
            CommandRtnCheck(rtn, "ZAux_Direct_SetOp ");

        }

        /// <summary>
        /// 关闭励磁。
        /// </summary>
        /// <param name="axis">轴号</param>
        protected override void ChildServoOff(int axis)
        {
            int rtn = zmcaux.ZAux_Direct_SetOp(m_cardHandle, 12 + axis, 0);
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
            int rtn = zmcaux.ZAux_Direct_GetMpos(m_cardHandle, axis, ref pos);
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
                rtn = zmcaux.ZAux_Direct_SetOp(m_cardHandle, port, 1);
            else
                rtn = zmcaux.ZAux_Direct_SetOp(m_cardHandle, port, 0);

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
            int rtn = zmcaux.ZAux_Direct_GetOp(m_cardHandle, port, ref pivalue);
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
           int rtn= zmcaux.ZAux_Direct_GetOutMulti(m_cardHandle, 0, 15, outputStatus);
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
            int rtn = zmcaux.ZAux_Direct_GetIn(m_cardHandle, port, ref pivalue);
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
            int rtn = zmcaux.ZAux_Direct_GetInMulti(m_cardHandle, 0, 15, outputStatus);
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
            throw new NotSupportedException();
        }

        /// <summary>
        /// 读取指定模拟输入端口的电压值。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <returns></returns>
        protected override double ChildReadAnalogInput(int port)
        {

            throw new Exception($"{PortName}  硬件不支持模拟量输入");
        }

        /// <summary>
        /// 读取所有模拟输出端口的电压值。
        /// </summary>
        /// <returns>电压值列表。</returns>
        protected override IReadOnlyList<double> ChildReadAnalogOutput()
        {
            throw new Exception($"{PortName}  硬件不支持模拟量输出");
        }

        /// <summary>
        /// 读取指定模拟输出端口的电压值。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <returns></returns>
        protected override double ChildReadAnalogOutput(int port)
        {
            throw new Exception($"{PortName}  硬件不支持模拟量输入");

        }

        /// <summary>
        /// 设置指定模拟输出端口的电压值。
        /// </summary>
        /// <param name="port">端口号</param>
        /// <param name="value">电压值</param>
        protected override void ChildSetAnalogOutput(int port, double value)
        {
            throw new Exception($"{PortName}  硬件不支持模拟量输出");


        }

        /// <summary>
        /// 打开指定模拟输出端口的输出。
        /// </summary>
        /// <param name="port">端口号</param>
        protected override void ChildAnalogOutputOn(int port)
        {
        }

        /// <summary>
        /// 关闭指定模拟输出端口的输出。
        /// </summary>
        /// <param name="port">端口号</param>
        protected override void ChildAnalogOutputOff(int port)
        {
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
            throw new NotSupportedException();
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
                rtn = zmcaux.ZAux_Direct_Single_Cancel(m_cardHandle, i, 2);
                CommandRtnCheck(rtn, "ZAux_Direct_Single_Cancel");
            }

        }

        /// <summary>
        /// 关闭运动控制器，并销毁运动控制器实例。
        /// </summary>
        protected override void ChildDispose()
        {
          int rtn=  zmcaux.ZAux_Close(m_cardHandle);
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
            int rtn = zmcaux.ZAux_Direct_GetAxisStatus(m_cardHandle, 0, ref axiaStatus);
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

        private void ReadParamFile(string filePath, ref McParam cardParam)
		{
            if(!File.Exists(filePath))
			{
                return;
			}
            string jsonInfo=   File.ReadAllText(filePath);
            try
			{
                cardParam = JsonConvert.DeserializeObject<McParam>(jsonInfo);
            }
            catch (FormatException ex)
			{
                throw new Exception($"{filePath} ,配置文件加载异常");
			}

		}

        private void LoadParam(IntPtr cardHandle, McParam cardParam)
		{
            int rtn;
            foreach (var mem in cardParam.AxisParams)
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

                if (mem.HomeParam.OrgIo >= -1)
                {
                    rtn = zmcaux.ZAux_Direct_SetDatumIn(cardHandle, mem.Index, mem.HomeParam.OrgIo);
                    CommandRtnCheck(rtn, nameof(zmcaux.ZAux_Direct_SetDatumIn));
                }
                if (mem.HomeParam.OrgIoInv)
                {
                    rtn = zmcaux.ZAux_Direct_SetInvertIn(cardHandle, mem.HomeParam.OrgIo, 1);
                }

                if (mem.HomeParam.PelIo >= -1)
                {
                    rtn = zmcaux.ZAux_Direct_SetFwdIn(cardHandle, mem.Index, mem.HomeParam.PelIo);
                    CommandRtnCheck(rtn, "ZAux_Direct_SetFwdIn in LoadParam Function");
                }
                if (mem.HomeParam.PelIoInv)
                {
                    rtn = zmcaux.ZAux_Direct_SetInvertIn(cardHandle, mem.HomeParam.PelIo, 1);
                }

                if (mem.HomeParam.NelIo >= -1)
                {
                    rtn = zmcaux.ZAux_Direct_SetRevIn(cardHandle, mem.Index, mem.HomeParam.NelIo);
                    CommandRtnCheck(rtn, "ZAux_Direct_SetFwdIn in LoadParam Function");
                }
                if (mem.HomeParam.NelIoInv)
                {
                    rtn = zmcaux.ZAux_Direct_SetInvertIn(cardHandle, mem.HomeParam.NelIo, 1);
                }
            }
		}

        private AxisParam FindAxisParam(int axis, ref McParam cardParam)
		{
            var param = cardParam.AxisParams.FirstOrDefault(x => x.Index == axis);
            if (param == null)
                throw new NullReferenceException($"无法在文件{_configFileAxis}中找到轴{axis}的配置参数。");

            return param;
		}


        public void UnitTest()
		{
            Init();
            ServoOn(0);
            Home(0, 100000, 10000);
            Move(0, 100000, 200000);
            ServoOff(0);
		}
    }
}
