import enum
import os
import locale
from ctypes import (
    CDLL,
    c_uint8,
    c_int32,
    c_uint32,
    c_double,
    byref,
    create_string_buffer,
    POINTER,
    pointer,
    cast,
    c_char,
    c_char_p,
)

if os.name == "nt":
    from ctypes import WinDLL


class MEDAQLibException(Exception):
    def __init__(self, function_name, iSensor, iRet):
        self.iSensor = iSensor
        self.iRet = iRet
        self.functionName = function_name

    def __str__(self):
        buffer = create_string_buffer(1024)
        MEDAQLib.GetError(self.iSensor, buffer)
        return "Calling function {0} returned {1} ({2})".format(
            self.functionName, self.iRet, buffer.value
        )


class ME_SENSOR(enum.IntEnum):
    """sensor constants from MEDAQLib.h"""

    NO_SENSOR = 0  # Dummy, only for internal use
    SENSOR_ILR1171 = 69  # optoNCDT ILR
    SENSOR_ILR118x = 20  # optoNCDT ILR
    SENSOR_ILR1191 = 21  # optoNCDT ILR
    SENSOR_ILR2250 = 60  # optoNCDT ILR
    SENSOR_ILD1220 = 56  # optoNCDT
    SENSOR_ILD1320 = 41  # optoNCDT
    SENSOR_ILD1420 = 42  # optoNCDT
    SENSOR_ILD1750 = 51  # optoNCDT
    SENSOR_ILD1900 = 58  # optoNCDT
    SENSOR_ILD2300 = 29  # optoNCDT
    SENSOR_IFD2410 = 64  # confocalDT
    SENSOR_IFD2411 = 65  # confocalDT
    SENSOR_IFD2415 = 66  # confocalDT
    SENSOR_IFD2416 = 67  # confocalDT
    SENSOR_IFD2421 = 46  # confocalDT
    SENSOR_IFD2422 = 47  # confocalDT
    SENSOR_IFD2445 = 39  # confocalDT
    SENSOR_IFD2451 = 30  # confocalDT
    SENSOR_IFD2461 = 44  # confocalDT
    SENSOR_IFD2465 = 62  # confocalDT
    SENSOR_IFD2466 = 63  # confocalDT
    SENSOR_IFD2471 = 26  # confocalDT
    SENSOR_ODC2500 = 8  # optoCONTROL
    SENSOR_ODC2520 = 37  # optoCONTROL
    SENSOR_ODC2600 = 9  # optoCONTROL
    SENSOR_ODC2700 = 70  # optoCONTROL
    SENSOR_LLT27xx = 31  # scanCONTROL + gapCONTROL, only for SensorFinder functionality, OpenSensor will fail
    SENSOR_DT3060 = 50  # eddyNCDT for whole DT306x and DT307x family
    SENSOR_DT3100 = 28  # eddyNCDT
    SENSOR_IMC5400 = 55  # interferoMETER
    SENSOR_IMC5600 = 59  # interferoMETER
    SENSOR_DT6120 = 40  # capaNCDT
    CONTROLLER_DT6200 = 33  # capaNCDT
    CONTROLLER_KSS6430 = 45  # capaNCDT
    CONTROLLER_DT6500 = 15  # capaNCDT
    SENSOR_ON_MEBUS = (
        43  # Generic sensor with MEbus protocol support, only for internal use
    )
    PCI_CARD_IF2008 = 22  # PCI card IF2008
    ETH_ADAPTER_IF2008 = 52  # IF2008 ethernet adapter
    ETH_IF1032 = 34  # Interface module Ethernet / EtherCAT
    USB_ADAPTER_IF2004 = 36  # IF2004 USB adapter(4 x RS422)
    CONTROLLER_CBOX = 38  # External C-Box controller
    THICKNESS_SENSOR = 48  # thicknessSENSOR
    CONTROLLER_DPU = 68  # External DPU controller
    SENSOR_ACS7000 = 35  # colorCONTROL
    SENSOR_CFO = 53  # colorSENSOR
    SENSOR_MFA = 61  # colorCONTROL
    MULTI_SENSOR = 57  # Container for synchronized data aquisition of multiple sensors
    NUMBER_OF_SENSORS = 70


class ERR_CODE(enum.IntEnum):
    """Error Constants form MEDAQLib ErrorCodes.h"""

    ERR_NOERROR = 0
    ERR_FUNTION_NOT_SUPPORTED = -1
    ERR_CANNOT_OPEN = -2
    ERR_NOT_OPEN = -3
    ERR_APPLYING_PARAMS = -4
    ERR_SEND_CMD_TO_SENSOR = -5
    ERR_CLEARING_BUFFER = -6
    ERR_HW_COMMUNICATION = -7
    ERR_TIMEOUT_READING_FROM_SENSOR = -8
    ERR_READING_SENSOR_DATA = -9
    ERR_INTERFACE_NOT_SUPPORTED = -10
    ERR_ALREADY_OPEN = -11
    ERR_CANNOT_CREATE_INTERFACE = -12
    ERR_NO_SENSORDATA_AVAILABLE = -13
    ERR_UNKNOWN_SENSOR_COMMAND = -14
    ERR_UNKNOWN_SENSOR_ANSWER = -15
    ERR_SENSOR_ANSWER_ERROR = -16
    ERR_SENSOR_ANSWER_TOO_SHORT = -17
    ERR_WRONG_PARAMETER = -18
    ERR_NOMEMORY = -19
    ERR_NO_ANSWER_RECEIVED = -20
    ERR_SENSOR_ANSWER_DOES_NOT_MATCH_COMMAND = -21
    ERR_BAUDRATE_TOO_LOW = -22
    ERR_OVERFLOW = -23
    ERR_INSTANCE_NOT_EXTST = -24
    ERR_NOT_FOUND = -25
    ERR_WARNING = -26
    ERR_SENSOR_ANSWER_WARNING = -27


class LOG_LEVEL(enum.IntEnum):
    DATA_LEVEL = (64,)  # ! < MTDebug
    TRACE_LEVEL = (32,)  # ! < MTInfo
    NOTICE_LEVEL = (16,)  # ! < MTNotice
    WARNING_LEVEL = (8,)  # ! < MTWarning
    ERROR_LEVEL = (4,)  # ! < MTCritical
    CRITICAL_LEVEL = (2,)  # ! < MTAlert
    EMERGENCY_LEVEL = (1,)  # ! < MTEmergency


class LOG_TYPE(enum.IntEnum):
    HIGH_TYPE = (1,)  # (User <--> MEDAQLib)
    MIDDLE_TYPE = (2,)  # (Sensor layer <--> Interface layer)
    LOW_TYPE = (4,)  # (MEDAQLib <--> Hardware driver)
    ERROR_TYPE = 8  # (Any errors reported by MEDAQLib)
    DRIVER_TYPE = 16  # (Hardware driver <--> Sys driver)
    APPL_TYPE = 32  # (Application specific, see LogToFile and LogToFileU)


class MEDAQLib:
    """load MEDAQLib.dll from module path"""

    medaq_lib = CDLL("libMEDAQLib.so")
    medaq_lib_cdecl = CDLL("libMEDAQLib.so")

    @staticmethod
    def CreateSensorInstance(Sensor):
        """Creates an instance of the specified sensor driver by it's sensor constant
           and returns an index greater 0. If the function fails, 0 is returned.

        :param Sensor: sensor constant
        :return: sensor index
        """
        CreateSensorInstanceOrg = MEDAQLib.medaq_lib.CreateSensorInstance
        CreateSensorInstanceOrg.argtypes = [
            c_int32,
        ]
        CreateSensorInstanceOrg.restype = c_int32
        handle = CreateSensorInstanceOrg(Sensor)
        return MEDAQLib(handle)

    @staticmethod
    def CreateSensorInstByName(sensorName):
        """Creates an instance of the specified sensor driver by it's sensor name
            and returns an index greater 0. If the function fails, 0 is returned.

        :param sensorName: sensor name
        :return: sensor index
        """
        CreatSensorInstanceByNameOrg = MEDAQLib.medaq_lib.CreateSensorInstByName
        CreatSensorInstanceByNameOrg.argtypes = [
            c_char_p,
        ]
        CreatSensorInstanceByNameOrg.restype = c_int32
        handle = CreatSensorInstanceByNameOrg(bytes(sensorName, "utf8"))
        return MEDAQLib(handle)

    # tested Ok.
    def GetDLLVersion(self):
        """Retrievs the version of the MEDAQLib dll.
        The version is stored in versionStr and is limited to length of maxLen (should be at least 64 bytes)

        :return: dll-version
        """
        version = create_string_buffer(20)
        self._last_error = self._get_dll_version_org(version, len(version))
        DLLVersion = "{0}".format(version.value)
        DLLVersion = str(DLLVersion.lstrip("b"))
        return str(DLLVersion.strip("'"))

    # tested Ok.
    def SetParameterInt(self, paramName, paramValue):
        """Specifies an 4 Byte integer parameter for a OpenSensor- or SensorCommand-function call.

        :param paramName: Name of the parameter as string
        :param paramValue: Value of the parameter
        :return: none
        """
        paramName = eval("b" + "'" + paramName + "'")
        paramValue = c_int32(paramValue)
        self._last_error = self._set_parameter_int_org(
            self.iSensor, paramName, paramValue
        )

    def SetParameterDWORD_PTR(self, paramName, paramValue):
        """Set a 4 Byte (Win32) or 8 Byte (Win64) unsigned integer parameter.

        :param paramName: Name of the parameter as string
        :param paramValue: Value of the parameter
        :return: none
        """
        self._last_error = self._set_parameter_dword_ptr_org(
            self.iSensor, bytes(paramName, "utf8"), paramValue
        )

    # tested OK
    def SetParameterDouble(self, paramName, paramValue):
        """Specifies a 8 Byte double parameter for a OpenSensor- or SensorCommand-function call.

        :param paramName: Name of the parameter as string
        :param paramValue: Value of the parameter
        :return: none
        """
        self._last_error = self._set_parameter_double_org(
            self.iSensor, bytes(paramName, "utf8"), paramValue
        )

    # tested Ok.
    def SetParameterString(self, paramName, paramValue):
        """Specifies a string parameter for a OpenSensor- or SensorCommand-function call.

        :param paramName: Name of the parameter as string
        :param paramValue: Value of the parameter
        :return: none
        """
        self._last_error = self._set_parameter_string_org(
            self.iSensor, bytes(paramName, "utf8"), bytes(paramValue, "utf8")
        )

    # tested OK.
    def SetParameterBinary(self, paramName, paramValue):
        """Specifies a binary data array parameter for a OpenSensor- or SensorCommand-function call.

        :param paramName: Name of the parameter as string
        :param paramValue: Value of the parameter
        :return: none
        """
        len_buffer = len(paramValue)
        param_value = (c_uint8 * len_buffer)()
        for i in range(len_buffer):
            param_value[i] = paramValue[i]
        self._last_error = self._set_parameter_binary_org(
            self.iSensor, bytes(paramName, "utf8"), param_value, c_uint32(len_buffer)
        )

    # testet OK.
    def SetParameters(self, parameter_list):
        """Sets a list of parameters at once

        :param parameter_list: list of parameters
        :return: none
        """
        self._last_error = self._set_parameter_string_org(
            self.iSensor, bytes(parameter_list, "utf8")
        )

    # tested Ok.
    def GetParameterInt(self, paramName):
        """Returns an 4 Byte integer parameter after calling SensorCommand.

        :param paramName: Name of the parameter as string
        :return: Integer return value
        """
        ret_int = c_int32()
        self._last_error = self._get_parameter_int_org(
            self.iSensor, bytes(paramName, " utf8"), byref(ret_int)
        )
        return ret_int.value

    def GetParameterDWORD_PTR(self, paramName):
        """Get a 4 Byte (Win32) or 8 Byte (Win64) unsigned integer parameter.

        :param paramName: Name of the parameter as string
        :return:Pointer to a variable retrieving the parameter
        """
        ret_ptr = POINTER(c_uint32)()
        self._last_error = self._get_parameter_dword_ptr_org(
            self.iSensor, bytes(paramName, "utf8"), byref(ret_ptr)
        )
        return ret_ptr

    # tested Ok.
    def GetParameterDouble(self, paramName):
        """Returns an 8 Byte double parameter after calling SensorCommand.

        :param paramName: Name of the parameter as string
        :return: double return value
        """
        double_val = c_double()
        self._last_error = self._get_parameter_double_org(
            self.iSensor, bytes(paramName, "utf8"), byref(double_val)
        )
        return double_val.value

    # testet OK.
    def GetParameterString(self, paramName, max_len):
        """Returns a string parameter after calling SensorCommand.

        :param paramName: Name of the parameter as string
        :param max_len: Maximal length of the receive buffer
        :return: string return value
        """
        len = c_uint32(max_len)
        param_value = create_string_buffer(max_len)
        self._last_error = self._get_parameter_string_org(
            self.iSensor, bytes(paramName, "utf8"), param_value, byref(len)
        )
        return str(param_value, "utf8")[: len.value]

    # testet OK.
    def GetParameterBinary(self, paramName, max_len):
        """Returns binary data after calling SensorCommand.

        :param paramName: Name of the parameter as string
        :param max_len: Maximal length of the receive buffer
        :return: binary return value
        """
        max_buffer_len = c_uint32(max_len)
        param_value = (c_uint8 * max_len)()
        self._last_error = self._get_parameter_binary_org(
            self.iSensor, bytes(paramName, "utf8"), param_value, byref(max_buffer_len)
        )
        return bytearray(param_value)

    # testet OK
    def GetParameters(self, max_len):
        """Get all available Parameters at once

        :param max_len: Maximal length of the receive buffer
        :return: parameter list (name and value separated by an equality sign)
        """
        max_buffer_len = c_uint32(max_len)
        param_value = create_string_buffer(max_len)
        self._last_error = self._get_parameters_org(
            self.iSensor, param_value, byref(max_buffer_len)
        )
        return str(param_value.raw, "utf8")[: max_buffer_len.value]

    # testet OK
    def ClearAllParameters(self):
        """Clears the internal buffer of parameters.

        :return: none
        """
        self._last_error = self._clear_all_parameters_org(self.iSensor)

    # tested Ok.
    def OpenSensor(self):
        """Establish a connection to the sensor using the interface and parameters specified at SetParameters.

        :return: none
        """
        self._last_error = self._open_sensor_org(self.iSensor)

    # testet OK.
    def CloseSensor(self):
        """Close the connection to the connected sensor.

        :return: none
        """
        self._last_error = self._close_sensor_org(self.iSensor)

    # testet OK.
    def ReleaseSensorInstance(self):
        """Free the specific sensor instance

        :return: none
        """
        self._last_error = self._release_sensor_instance_org(self.iSensor)

    # tested Ok.
    def SensorCommand(self):
        """Send a command to the sensor and retries the answer.
        Both, command and answer can be set/read with Set- and GetParameter.

        :return: none
        """
        self._last_error = self._sensor_command_org(self.iSensor)

    # tested Ok.
    def DataAvail(self):
        """Check if data from Sensor si available and returns the number of values.

        :return: size of available data
        """
        dataAvailParam = c_int32(0)
        self._last_error = self._data_avail_org(self.iSensor, byref(dataAvailParam))
        return dataAvailParam.value

    # tested Ok.
    def TransferData(self, maxValues):
        """Transfer the data form driver to application.

        :param maxValues: Length of rawData and scaledData.
        :return: a tupel with the raw data, the scaled data and the real number of transfered data values
        """
        return self._transferdata_helper(maxValues, False)

    def TransferDataTs(self, maxValues):
        """Same as TransferData but with an additional parameter to retrieve times-
            tamp of data.

        :param maxValues: Length of rawData and scaledData.
        :return: a tupel with the raw data, the scaled data and the real number of transfered data values, such as the timestamp
        """
        return self._transferdata_helper(maxValues, True)

    # tested Ok.
    def Poll(self, maxValues):
        """Retrievs specified amount of values from Sensor (max. one frame).

        :param maxValues: Length of rawData and scaledData.
        :return: a tupel with rawData and scaledData
        """
        iValue = (c_int32 * maxValues)()
        dValue = (c_double * maxValues)()
        self._last_error = self._poll_org(
            self.iSensor,
            cast(iValue, POINTER(c_int32)),
            cast(dValue, POINTER(c_double)),
            maxValues,
        )

        raw_data = [i for i in iValue]
        scaled_data = [i for i in dValue]
        return (raw_data, scaled_data)

    def GetLastError(self):
        """Returns the return code of the function which was called last

        :return: errorcode of last function
        """
        return c_int32(self._last_error).value

    # testet OK.
    def GetError(self, maxLen=1024):
        """If an error had occured, the error text can be retrieved with GetError.

        :param buffer: stringbuffer to get extened error string
        :return: buffer with ErrorCode as string
        """
        len = c_uint32(maxLen)
        buffer = create_string_buffer(maxLen)
        self._last_error = self._get_error_org(self.iSensor, buffer, len)
        try:
            return str(buffer, "utf8")[: len.value]
        except:
            # Error messages may contain Windows language specific information, so try a 2nd time
            return str(buffer, locale.getpreferredencoding())[: len.value]

    def EnableLogging(
        self,
        enableLogging,
        logType,
        logLevel,
        logFile,
        logAppend,
        logFlush,
        logSplitSize,
    ):
        """

        :param enableLogging: This paramenter enables or disables logging to file for
        :param logType: This paramenter specifies the type of messages to log.
        :param logLevel: This paramenter specifies the kind of event to log.
        :param logFile: File name of log file.
        :param logAppend: This paramenter specifies if the logfile should be cleared at
                            opening or if the new data should be appended to file.
        :param logFlush: This paramenter specifies if the logfile should be flushed
                        after each output.
        :param logSplitSize: If this parameter is greater than 0, logfile is closed and
                        reopened when this size is reached.
        :return: none
        """
        self._last_error = self._enable_logging_org(
            self.iSensor,
            enableLogging,
            logType,
            logLevel,
            bytes(logFile, "utf8"),
            logAppend,
            logFlush,
            logSplitSize,
        )

    def LogToFile(self, logLevel, location, message, argumentList=""):
        """Add a line to MEDAQLib Logfile.

        :param logLevel: This paramenter specifies the level for the line to log.
        :param location: Location in source code where the log line is generated. Is
        :param message: Logging message. This parameter can be used at same as

        :return: none
        """
        ref = pointer(c_char())
        self._last_error = self._log_to_file_org(
            self.iSensor, logLevel, bytes(location, "utf8"), bytes(message, "utf8"), ref
        )

    def OpenSensorRS232(self, port):
        """Opens a Sensor on a specific interface

        :param port: Name of the serial interface. Before opening the interface
                    using CreateFile, the string is prefixed with "nn.n".
        :return: none
        """
        self._last_error = self._open_sensor_rs232_org(
            self.iSensor, bytes(port, "utf8")
        )

    def OpenSensorIF2004_USB(self, deviceInstance, serialNumber, port, channelNumber):
        """Set the parameters for USB adpater IF2004 before calling OpenSensor.

        :param deviceInstance: Instance number of the USB adapter IF2004.
        :param serialNumber: Serial number of the USB adapter (optional).
        :param port: Name of the serial interface part of USB adapter, e.g.
                    COM1, COM2, ... (optional).
        :param channelNumber: Channel number on USB adapeter IF2004.
        :return: none
        """
        self._last_error = self._open_sensor_if2004_usb_org(
            self.iSensor,
            deviceInstance,
            bytes(serialNumber, "utf8"),
            bytes(port, "utf8"),
            channelNumber,
        )

    def OpenSensorIf2008(self, cardNumber, channelNumber):
        """Set the parameters for IF2008 interface card before calling OpenSensor

        :param cardNumber: Instance number of the IF2008 interface card.
        :param channelNumber: Channel number on IF2008 Interface card.
        :return: none
        """
        self._last_error = self._open_sensor_if2008_org(
            self.iSensor, cardNumber, channelNumber
        )

    def OpenSensorTCPIP(self, remoteAddr):
        """Set the parameters for TCP/IP ethernet interface befor calling OpenSensor

        :param remoteAddr: IP address of the remote sensor (TCP server).
        :return: none
        """
        self._last_error = self._open_sensor_tcp_ip_org(
            self.iSensor, bytes(remoteAddr, "utf8")
        )

    def OpenSensorWinUSB(self, deviceInstance):
        """Set the parameters for USB interface via WinUSB before calling OpenSensor

        :param deviceInstance: Instance number of the USB device.
        :return: none
        """
        self._last_error = self._open_sensor_win_usb_org(self.iSensor, deviceInstance)

    def ExecSCmd(self, sensorCommand):
        """Set the sensor command name and executes the sensor command.

        :param sensorCommand: Name of the sensor command (used for parameter S_Command)
        :return: none
        """
        self._last_error = self._exec_s_cmd_org(
            self.iSensor, bytes(sensorCommand, "utf8")
        )

    def SetIntExecSCmd(self, sensorCommand, paramName, paramValue):
        """Set the sensor command name and an integer parameter and executes teh sensor command

        :param sensorCommand: Name of the sensor command (used for parameter S_Command)
        :param paramName: Name of the parameter as string
        :param paramValue: Int value of the parameter
        :return: none
        """
        self._last_error = self._set_int_exec_s_cmd_org(
            self.iSensor,
            bytes(sensorCommand, "utf8"),
            bytes(paramName, "utf8"),
            paramValue,
        )

    def SetDoubleExecSCmd(self, sensorCommand, paramName, paramValue):
        """Set the sensor command name and a double parameter and executes teh sensor command

        :param sensorCommand: Name of the sensor command (used for parameter S_Command)
        :param paramName: Name of the parameter as string
        :param paramValue: Double value of the parameter
        :return: none
        """
        self._last_error = self._set_double_exec_cmd_org(
            self.iSensor,
            bytes(sensorCommand, "utf8"),
            bytes(paramName, "utf8"),
            paramValue,
        )

    def SetStringExecSCmd(self, sensorCommand, paramName, paramValue):
        """Set the sensor command name and a string parameter and executes teh sensor command

        :param sensorCommand: Name of the sensor command (used for parameter S_Command)
        :param paramName: Name of the parameter as string
        :param paramValue: String value of the parameter
        :return: none
        """
        self._last_error = self._set_string_exec_cmd_org(
            self.iSensor,
            bytes(sensorCommand, "utf8"),
            bytes(paramName, "utf8"),
            bytes(paramValue, "utf8"),
        )

    def ExecSCmdGetInt(self, sensorCommand, paramName):
        """Set the sensor command name, executes the sensor command and get an integer parameter

        :param sensorCommand: Name of the sensor command (used for parameter S_Command)
        :param paramName: Name of the parameter as string
        :return: returned integer parameter
        """
        ret_int = c_int32()
        self._last_error = self._exec_cmd_get_int_org(
            self.iSensor,
            bytes(sensorCommand, "utf8"),
            bytes(paramName, "utf8"),
            byref(ret_int),
        )
        return ret_int.value

    def ExecSCmdGetDouble(self, sensorCommand, paramName):
        """Set the sensor command name, executes the sensor command and get a double parameter

        :param sensorCommand: Name of the sensor command (used for parameter S_Command)
        :param paramName: Name of the parameter as string
        :return: returned double parameter
        """
        ret_double = c_double()
        self._last_error = self._exec_cmd_get_double_org(
            self.iSensor,
            bytes(sensorCommand, "utf8"),
            bytes(paramName, "utf8"),
            byref(ret_double),
        )
        return ret_double.value

    def ExecSCmdGetString(self, sensorCommand, paramName, maxLength):
        """Set the sensor command name, executes the sensor command and get a string parameter

        :param sensorCommand: Name of the sensor command (used for parameter S_Command)
        :param paramName: Name of the parameter as string
        :return: returned string parameter
        """
        len = c_uint32(maxLength)
        param_buffer = create_string_buffer(maxLength)
        self._last_error = self._exec_cmd_get_string_org(
            self.iSensor,
            bytes(sensorCommand, "utf8"),
            bytes(paramName, "utf8"),
            param_buffer,
            byref(len),
        )
        return str(param_buffer, "utf8")[: len.value]

    def _transferdata_helper(self, maxValues, timeStampEnable):
        """helper method for transferdata and transferdataTS if timestamp == -1 use transferdata else transferdataTS

        :param maxValues: maximal number of values to return
        :return: a tupel with the raw data, the scaled data and the real number of transfered data values
                if transfer_data_ts is called the timestamp is appended
        """
        if maxValues >= 0:
            raw_data_buffer = (c_int32 * maxValues)()
            scaled_data_buffer = (c_double * maxValues)()
            maxValues = c_int32(maxValues)
            read = c_int32()
            timeStamp = c_double()
            if timeStampEnable is False:
                self._last_error = self._transfer_data_org(
                    self.iSensor,
                    cast(raw_data_buffer, POINTER(c_int32)),
                    cast(scaled_data_buffer, POINTER(c_double)),
                    maxValues,
                    byref(read),
                )
            else:
                self._last_error = self._transfer_data_ts_org(
                    self.iSensor,
                    cast(raw_data_buffer, POINTER(c_int32)),
                    cast(scaled_data_buffer, POINTER(c_double)),
                    maxValues,
                    byref(read),
                    byref(timeStamp),
                )
            raw_data = list()
            scaled_data = list()
            for i in range(read.value):
                raw_data.append(raw_data_buffer[i])
                scaled_data.append(scaled_data_buffer[i])
            if timeStampEnable is False:
                return (raw_data, scaled_data, read.value)
            else:
                return (raw_data, scaled_data, read.value, timeStamp.value)
        return c_int32(0)

    def __init__(self, handle):
        """Constructor sets the sensor handle and inits the MEDAQLib function

        :param handle: instance handle of the connected sensor
        """
        self.iSensor = handle
        self._last_error = c_int32(0)
        self.init_sensor_functions()

    def __del__(self):
        """Destructor closes the sensor connection and releases the handle"""
        self.CloseSensor()
        self.ReleaseSensorInstance()

    def init_sensor_functions(self):
        """Defines and initializes MEDAQLib functions for the python wrapper.

        :return: none
        """

        # GetDllVersion
        self._get_dll_version_org = MEDAQLib.medaq_lib.GetDLLVersion
        self._get_dll_version_org.argtypes = [c_char_p, c_uint32]
        self._get_dll_version_org.restype = c_int32

        # SetParameterInt
        self._set_parameter_int_org = MEDAQLib.medaq_lib.SetParameterInt
        self._set_parameter_int_org.argtypes = [c_uint32, c_char_p, c_int32]
        self._set_parameter_int_org.restype = c_int32

        # SetParameterDWORD_PTR
        self._set_parameter_dword_ptr_org = MEDAQLib.medaq_lib.SetParameterDWORD_PTR
        self._set_parameter_dword_ptr_org.argtypes = [
            c_uint32,
            c_char_p,
            POINTER(c_uint32),
        ]
        self._set_parameter_dword_ptr_org.restype = c_int32

        # SetParameterDouble
        self._set_parameter_double_org = MEDAQLib.medaq_lib.SetParameterDouble
        self._set_parameter_double_org.argtypes = [c_uint32, c_char_p, c_double]
        self._set_parameter_double_org.restype = c_int32

        # SetParameterString
        self._set_parameter_string_org = MEDAQLib.medaq_lib.SetParameterString
        self._set_parameter_string_org.argtypes = [c_uint32, c_char_p, c_char_p]
        self._set_parameter_string_org.restype = c_int32

        # SetParameterBinary
        self._set_parameter_binary_org = MEDAQLib.medaq_lib.SetParameterBinary
        self._set_parameter_binary_org.argtypes = [
            c_uint32,
            c_char_p,
            POINTER(c_uint8),
            c_uint32,
        ]
        self._set_parameter_binary_org.restype = c_int32

        # GetParameterInt
        self._get_parameter_int_org = MEDAQLib.medaq_lib.GetParameterInt
        self._get_parameter_int_org.argtypes = [c_uint32, c_char_p, POINTER(c_int32)]
        self._get_parameter_int_org.restype = c_int32

        # GetParameterDWORD_PTR
        self._get_parameter_dword_ptr_org = MEDAQLib.medaq_lib.GetParameterDWORD_PTR
        self._get_parameter_dword_ptr_org.argtypes = [
            c_uint32,
            c_char_p,
            POINTER(POINTER(c_uint32)),
        ]
        self._get_parameter_dword_ptr_org.restype = c_int32

        # GetParameterDouble
        self._get_parameter_double_org = MEDAQLib.medaq_lib.GetParameterDouble
        self._get_parameter_double_org.argtypes = [
            c_uint32,
            c_char_p,
            POINTER(c_double),
        ]
        self._get_parameter_double_org.restype = c_int32

        # GetParameterString
        self._get_parameter_string_org = MEDAQLib.medaq_lib.GetParameterString
        self._get_parameter_string_org.argtypes = [
            c_uint32,
            c_char_p,
            c_char_p,
            POINTER(c_uint32),
        ]
        self._get_parameter_string_org.restype = c_int32

        # GetParameterBinary
        self._get_parameter_binary_org = MEDAQLib.medaq_lib.GetParameterBinary
        self._get_parameter_binary_org.argtypes = [
            c_uint32,
            c_char_p,
            POINTER(c_uint8),
            POINTER(c_uint32),
        ]
        self._get_parameter_binary_org.restype = c_int32

        # GetParameters
        self._get_parameters_org = MEDAQLib.medaq_lib.GetParameters
        self._get_parameters_org.argtypes = [c_uint32, c_char_p, POINTER(c_uint32)]
        self._get_parameters_org.resttype = c_int32

        # SetParameters
        self._set_parameters_org = MEDAQLib.medaq_lib.SetParameters
        self._set_parameters_org.argtypes = [
            c_uint32,
            c_char_p,
        ]
        self._set_parameters_org.restype = c_int32

        # ClearAllParameters
        self._clear_all_parameters_org = MEDAQLib.medaq_lib.ClearAllParameters
        self._clear_all_parameters_org.argtypes = [
            c_uint32,
        ]
        self._clear_all_parameters_org.restype = c_int32

        # OpenSensor
        self._open_sensor_org = MEDAQLib.medaq_lib.OpenSensor
        self._open_sensor_org.argtypes = [
            c_uint32,
        ]
        self._open_sensor_org.restype = c_int32

        # CloseSensor
        self._close_sensor_org = MEDAQLib.medaq_lib.CloseSensor
        self._close_sensor_org.argtypes = [
            c_uint32,
        ]
        self._close_sensor_org.restype = c_int32

        # ReleaseSensorInstance
        self._release_sensor_instance_org = MEDAQLib.medaq_lib.ReleaseSensorInstance
        self._release_sensor_instance_org.argtypes = [
            c_uint32,
        ]
        self._release_sensor_instance_org.restype = c_int32

        # SensorCommand
        self._sensor_command_org = MEDAQLib.medaq_lib.SensorCommand
        self._sensor_command_org.argtypes = [
            c_uint32,
        ]
        self._sensor_command_org.restype = c_int32

        # Poll
        self._poll_org = MEDAQLib.medaq_lib.Poll
        self._poll_org.argtypes = [
            c_uint32,
            POINTER(c_int32),
            POINTER(c_double),
            c_int32,
        ]
        self._poll_org.restype = c_int32

        # DataAvail
        self._data_avail_org = MEDAQLib.medaq_lib.DataAvail
        self._data_avail_org.argtypes = [c_uint32, POINTER(c_int32)]
        self._data_avail_org.restype = c_int32

        # TransferData
        self._transfer_data_org = MEDAQLib.medaq_lib.TransferData
        self._transfer_data_org.argtypes = [
            c_uint32,
            POINTER(c_int32),
            POINTER(c_double),
            c_int32,
            POINTER(c_int32),
        ]
        self._transfer_data_org.restype = c_int32

        # TransferDataTS
        self._transfer_data_ts_org = MEDAQLib.medaq_lib.TransferDataTS
        self._transfer_data_ts_org.argtypes = [
            c_uint32,
            POINTER(c_int32),
            POINTER(c_double),
            c_int32,
            POINTER(c_int32),
            POINTER(c_double),
        ]
        self._transfer_data_ts_org.restype = c_int32

        # EnableLogging
        self._enable_logging_org = MEDAQLib.medaq_lib.EnableLogging
        self._enable_logging_org.argtypes = [
            c_uint32,
            c_int32,
            c_int32,
            c_int32,
            c_char_p,
            c_int32,
            c_int32,
            c_int32,
        ]
        self._enable_logging_org.restype = c_int32

        # LogtoFile
        self._log_to_file_org = MEDAQLib.medaq_lib_cdecl.LogToFile
        self._log_to_file_org.argtypes = [
            c_uint32,
            c_int32,
            c_char_p,
            c_char_p,
            POINTER(c_char),
        ]
        self._log_to_file_org.restype = c_int32

        # OpenSensorRS232
        self._open_sensor_rs232_org = MEDAQLib.medaq_lib.OpenSensorRS232
        self._open_sensor_rs232_org.argtypes = [c_uint32, c_char_p]
        self._open_sensor_rs232_org.restype = c_int32

        # OpenSensorIF2004_USB
        self._open_sensor_if2004_usb_org = MEDAQLib.medaq_lib.OpenSensorIF2004_USB
        self._open_sensor_if2004_usb_org.argtypes = [
            c_uint32,
            c_int32,
            c_char_p,
            c_char_p,
            c_int32,
        ]
        self._open_sensor_if2004_usb_org.restype = c_int32

        # OpenSensorIF2008
        self._open_sensor_if2008_org = MEDAQLib.medaq_lib.OpenSensorIF2008
        self._open_sensor_if2008_org.argtypes = [c_uint32, c_int32, c_int32]
        self._open_sensor_if2008_org.restype = c_int32

        # OpenSensorTCPIP
        self._open_sensor_tcp_ip_org = MEDAQLib.medaq_lib.OpenSensorTCPIP
        self._open_sensor_tcp_ip_org.argtypes = [c_uint32, c_char_p]
        self._open_sensor_tcp_ip_org.restype = c_uint32

        # OpenSensorWinUSB
        self._open_sensor_win_usb_org = MEDAQLib.medaq_lib.OpenSensorWinUSB
        self._open_sensor_win_usb_org.argtypes = [c_uint32, c_int32]
        self._open_sensor_win_usb_org.restype = c_int32

        # ExecSCmd
        self._exec_s_cmd_org = MEDAQLib.medaq_lib.ExecSCmd
        self._exec_s_cmd_org.argypes = [c_uint32, c_char_p]
        self._exec_s_cmd_org.restype = c_int32

        # SetIntExecSCmd
        self._set_int_exec_s_cmd_org = MEDAQLib.medaq_lib.SetIntExecSCmd
        self._set_int_exec_s_cmd_org.argtypes = [c_uint32, c_char_p, c_char_p, c_int32]
        self._set_int_exec_s_cmd_org.restype = c_int32

        # SetDoubleExecSCmd
        self._set_double_exec_cmd_org = MEDAQLib.medaq_lib.SetDoubleExecSCmd
        self._set_double_exec_cmd_org.argtypes = [
            c_uint32,
            c_char_p,
            c_char_p,
            c_double,
        ]
        self._set_double_exec_cmd_org.restype = c_int32

        # SetStringExecSCmd
        self._set_string_exec_cmd_org = MEDAQLib.medaq_lib.SetStringExecSCmd
        self._set_string_exec_cmd_org.argtypes = [
            c_uint32,
            c_char_p,
            c_char_p,
            c_char_p,
        ]
        self._set_string_exec_cmd_org.restype = c_int32

        # ExecSCmdGetInt
        self._exec_cmd_get_int_org = MEDAQLib.medaq_lib.ExecSCmdGetInt
        self._exec_cmd_get_int_org.argtypes = [
            c_uint32,
            c_char_p,
            c_char_p,
            POINTER(c_int32),
        ]
        self._exec_cmd_get_int_org.restype = c_int32

        # ExecSCmdGetDouble
        self._exec_cmd_get_double_org = MEDAQLib.medaq_lib.ExecSCmdGetDouble
        self._exec_cmd_get_double_org.argtypes = [
            c_uint32,
            c_char_p,
            c_char_p,
            POINTER(c_double),
        ]
        self._exec_cmd_get_double_org.restype = c_int32

        # ExecSCmdGetInt
        self._exec_cmd_get_string_org = MEDAQLib.medaq_lib.ExecSCmdGetString
        self._exec_cmd_get_string_org.argtypes = [
            c_uint32,
            c_char_p,
            c_char_p,
            c_char_p,
            POINTER(c_uint32),
        ]
        self._exec_cmd_get_string_org.restype = c_int32

        # GetError
        self._get_error_org = MEDAQLib.medaq_lib.GetError
        self._get_error_org.argtypes = [c_uint32, c_char_p, c_uint32]
        self._get_error_org.restype = c_int32
