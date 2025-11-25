#include "usbd_composite.h"

USBD_CDC_HandleTypeDef *pCDCData;
USBD_HID_HandleTypeDef *pHIDData;

static uint8_t USBD_Composite_Init(USBD_HandleTypeDef *pdev,
                                   uint8_t cfgidx);
static uint8_t USBD_Composite_DeInit(USBD_HandleTypeDef *pdev,
                                     uint8_t cfgidx);
static uint8_t USBD_Composite_Setup(USBD_HandleTypeDef *pdev,
                                    USBD_SetupReqTypedef *req);
static uint8_t USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_Composite_DataIn(USBD_HandleTypeDef *pdev,
                                     uint8_t epnum);
static uint8_t USBD_Composite_DataOut(USBD_HandleTypeDef *pdev,
                                      uint8_t epnum);
static uint8_t *USBD_Composite_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_Composite_GetDeviceQualifierDescriptor(uint16_t *length);

USBD_ClassTypeDef USBD_COMPOSITE =
    {
        USBD_Composite_Init,
        USBD_Composite_DeInit,
        USBD_Composite_Setup,
        NULL,                       /*EP0_TxSent*/
        USBD_Composite_EP0_RxReady, // add
        USBD_Composite_DataIn,
        USBD_Composite_DataOut,
        NULL,
        NULL,
        NULL,
        NULL,
        USBD_Composite_GetFSCfgDesc,
        NULL,
        USBD_Composite_GetDeviceQualifierDescriptor,
};

/* USB composite device Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
__ALIGN_BEGIN uint8_t USBD_Composite_CfgFSDesc[USBD_COMPOSITE_DESC_SIZE] __ALIGN_END =
{
        0x09,                                   /* 配置描述符长度，固定为 9 字节*/
        USB_DESC_TYPE_CONFIGURATION,            /* 描述符类型，0x02，表示配置描述符 */
        USBD_COMPOSITE_DESC_SIZE,               /*配置及其所有接口和端点描述符的总长度（低字节和高字节，实际值由宏定义）必须对*/
        0x00,                                   
        USBD_MAX_NUM_INTERFACES,                /* bNumInterfaces 该配置下的接口数量（如 3，表示 CHID+CDC CMD+CDC DATA）: */
        0x01,                                   /* bConfiguration Value配置值，用于主机选择配置 允许一个设备支持多种配置（ed，低功耗，高速，不同功能组合等） */
        0x00,                                   /* iConfiguration 字符串索引 配置字符串描述符索引，0 表示无。 */
        0x80,                                   /* 电源属性，0x80 表示总线供电，不支持远程唤醒*/
        0x32,                                   /* 最大电流，单位为 2mA，0x32=50*2=100mA  */

        /****************************HID************************************/
        /* Interface Association Descriptor */
        USBD_IAD_DESC_SIZE,       // bLength IADIAD（接口关联描述符）的长度，固定为 8 字节
        USBD_IAD_DESCRIPTOR_TYPE, // bDescriptorType 描述符类型，0x0B，表示这是一个 IAD（Interface Association Descriptor）。
        0x00,                     // bFirstInterface CHID（自定义HID）相关的接口在整个配置描述符中的第一个接口（编号为 0），后续的接口编号会递增（如 CDC 的命令接口是 1，数据接口是 2）。
        0x01,                     // bInterfaceCount 接口描述符数量
        0x03,                     // bFunctionClass  功能类代码，0x03 表示 HID（Human Interface Device）类。
        0x00,                     // bFunctionSubClass  功能子类代码，0x00 表示无特定子类（标准 HID）。
        0x00,                     // bInterfaceProtocol 协议代码，0x00 表示无特定协议（如键盘为 0x01，鼠标为 0x02，这里为自定义）。
        0x00,                     // iFunction 字符串描述符索引，0 表示无字符串描述符。      

        /********************  HID interface ********************/
        /************** Descriptor of Custom HID interface ****************/
        /* 09 */
        0x09,                    /*bLength: 接口描述符长度，固定为 9 字节*/
        USB_DESC_TYPE_INTERFACE, /*bDescriptorType: 描述符类型，0x04，表示接口描述符*/
        USBD_HID_INTERFACE,      /*bInterfaceNumber: /*接口编号，HID接口在本配置中的编号为 0。 */
        0x00,                    /*bAlternateSetting: 备用设置编号，通常为 0，表示没有备用接口 */
        0x01,                    /*bNumEndpoints 该接口使用的端点数量，这里为 1（通常为 IN 端点）。*/
        0x03,                    /*bInterfaceClass: 接口类代码，0x03 表示 HID（Human Interface Device）类*/
        0x00,                    /*bInterfaceSubClass : 子类代码，0x00 表示无特殊子类（标准 HID）*/
        0x00,                    /*nInterfaceProtocol : 协议代码，0x00 表示无特定协议（如键盘为 0x01，鼠标为 0x02，这里为自定义）*/
        0,                       /*iInterface: Index of string descriptor*/

        /******************** Descriptor of Custom HID ********************/
        /* 18 */
        0x09,                /*bLength: HID类描述符长度，固定为 9 字节*/
        HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID描述符类型，0x21，表示 HID 类描述符*/
        0x00,                /*bcdHID: HID 规范版本号，0x0100，表示 HID 1.00*/
        0x01,
        0x00,                       /*bCountryCode: 国家代码，0x00 表示无特定国家*/
        0x01,                       /*bNumDescriptors: 后续 HID 类描述符数量，通常为 1（即后面跟一个报告描述符）*/
        0x22,                       /*bDescriptorType下一个描述符类型，0x22 表示报告描述符（Report Descriptor）*/
        HID_MOUSE_REPORT_DESC_SIZE, /*wItemLength: 报告描述符长度（低字节和高字节，实际值由宏定义），主机据此分配缓冲区*/
        0x00,
        /******************** Descriptor of TouchScreen endpoint ********************/
        /* 27 */
        0x07,                   /*bLength: 端点描述符长度，固定为 7 字节*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:描述符类型，0x05，表示端点描述符*/

        HID_EPIN_ADDR, /*bEndpointAddress: 端点地址，通常为 IN 端点（如 0x83），高位表示方向（IN），低位为端点号*/
        0x03,          /*bmAttributes: 端点属性，0x03 表示中断端点（Interrupt）*/
        HID_EPIN_SIZE, /*wMaxPacketSize: 最大包长（wMaxPacketSize），低字节和高字节*/
        0x00,
        HID_FS_BINTERVAL, /*bInterval: Polling Interval 轮询间隔（bInterval），单位为毫秒，决定主机查询该端点的频率（如 10ms） */
        /* 34 */

        /****************************CDC************************************/
        /* IAD??? */
        /* Interface Association Descriptor */
        USBD_IAD_DESC_SIZE,       // bLength
        USBD_IAD_DESCRIPTOR_TYPE, // bDescriptorType
        0x01,                     // bFirstInterface 接口描述符是在总的配置描述符中的第几个接口开始，这里 CDC 的命令接口编号是 1
        0x02,                     // bInterfaceCount 接口数量，这里 CDC 有两个接口：命令接口和数据接口
        0x02,                     // bFunctionClass     CDC Control
        0x02,                     // bFunctionSubClass  Abstract Control Model
        0x01,                     // bInterfaceProtocol  AT Commands: V.250 etc
        0x00,                     // iFunction

        /* CDC命令接口描述符 */
        /*Interface Descriptor */
        0x09,                    /* bLength: Interface Descriptor size ?? */
        USB_DESC_TYPE_INTERFACE, /* bDescriptorType: Interface 描述符类型，0x04 */
        /* Interface descriptor type */
        USBD_CDC_CMD_INTERFACE, /* bInterfaceNumber: Number of Interface Number of Interface 接口编号，第一个接口编号为1 */
        0x00,                   /* bAlternateSetting: Alternate setting 接口备用编号 0 */
        0x01,                   /* bNumEndpoints: One endpoints used 端点数量，这里为 1，CDC 命令接口通常只有一个中断端点 */
        0x02,                   /* bInterfaceClass: Communication Interface Class 通信接口类，0x02 表示 CDC */
        0x02,                   /* bInterfaceSubClass: Abstract Control Model 抽象控制模型，0x02 */
        0x01,                   /* bInterfaceProtocol: Common AT commands 通用 AT 命令协议，0x01 */
        0x00,                   /* iInterface: 接口字符串描述符索引，通常为 0 表示无字符串描述符 */

        /* 功能描述符--头部功能描述符 */
        /*Header Functional Descriptor*/
        0x05, /* bLength: Endpoint Descriptor size 5字节 */
        0x24, /* bDescriptorType: CS_INTERFACE 类别特定接口描述符 */
        0x00, /* bDescriptorSubtype: Header Func Desc 头部功能描述符，值为0x00 */
        0x10, /* bcdCDC: spec release number CDC规范版本号 */
        0x01,

        /*Call Management Functional Descriptor*/
        0x05, /* bFunctionLength */
        0x24, /* bDescriptorType: CS_INTERFACE ???????????CS_INTERFACE*/
        0x01, /* bDescriptorSubtype: Call Management Func Desc ???Call Management Func Desc ??0x01*/
        0x00, /* bmCapabilities: D0+D1 ???????call management */
        0x01, /* bDataInterface: 1 ??????????call management */

        /*ACM Functional Descriptor*/
        0x04, /* bFunctionLength */
        0x24, /* bDescriptorType: CS_INTERFACE ???????????CS_INTERFACE*/
        0x02, /* bDescriptorSubtype: Abstract Control Management desc ???Abstract Control Management desc??0x02*/
        0x02, /* bmCapabilities ??Set_Control_Line_State?Get_Line_Coding???Serial_State??*/

        /*Union Functional Descriptor*/
        0x05,                    /* bFunctionLength */
        0x24,                    /* bDescriptorType: CS_INTERFACE ???????????CS_INTERFACE */
        0x06,                    /* bDescriptorSubtype: Union func desc ???Union func desc ??0x06*/
        USBD_CDC_CMD_INTERFACE,  /* bMasterInterface: Communication class interface ???1?CDC?? */
        USBD_CDC_DATA_INTERFACE, /* bSlaveInterface0: Data Class Interface ???2?????? */

        /*Endpoint 2 Descriptor*/
        0x07,                        /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
        CDC_CMD_EP,                  /* bEndpointAddress */
        0x03,                        /* bmAttributes: Interrupt */
        LOBYTE(CDC_CMD_PACKET_SIZE), /* wMaxPacketSize: */
        HIBYTE(CDC_CMD_PACKET_SIZE),
        CDC_FS_BINTERVAL, /* bInterval: */
        /*---------------------------------------------------------------------------*/

        /* ??????????? */
        /*Data class interface descriptor*/
        0x09,                    /* bLength: Endpoint Descriptor size ???????9??*/
        USB_DESC_TYPE_INTERFACE, /* bDescriptorType: ????????0x04*/
        USBD_CDC_DATA_INTERFACE, /* bInterfaceNumber: Number of Interface ??????2*/
        0x00,                    /* bAlternateSetting: Alternate setting ?????????0 */
        0x02,                    /* bNumEndpoints: Two endpoints used ?0????? ????????????,???2*/
        0x0A,                    /* bInterfaceClass: CDC ???????? ????????0x0A */
        0x00,                    /* bInterfaceSubClass: ?????????0*/
        0x00,                    /* bInterfaceProtocol: ?????????0*/
        0x00,                    /* iInterface:  ?????????,0????*/

        /* ?????????? */
        /*Endpoint OUT Descriptor*/
        0x07,                                /* bLength: Endpoint Descriptor size ???????7?? */
        USB_DESC_TYPE_ENDPOINT,              /* bDescriptorType: Endpoint ????????0x05 */
        CDC_OUT_EP,                          /* bEndpointAddress ?????0x02 D7???*/
        0x02,                                /* bmAttributes: Bulk ????*/
        LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), /* wMaxPacketSize: ??????? 512??*/
        HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
        0x00, /* bInterval: ignore for Bulk transfer ??????,??????? */

        /* ?????????? */
        /*Endpoint IN Descriptor*/
        0x07,                                /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_ENDPOINT,              /* bDescriptorType: Endpoint ????????0x05*/
        CDC_IN_EP,                           /* bEndpointAddress ?????0x82 D7???*/
        0x02,                                /* bmAttributes: Bulk ????*/
        LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE), /* wMaxPacketSize: ??????? 512??*/
        HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
        0x00 /* bInterval: ignore for Bulk transfer ??????,???????*/
};

/* USB ???????? */
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_Composite_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
    {
        USB_LEN_DEV_QUALIFIER_DESC,
        USB_DESC_TYPE_DEVICE_QUALIFIER,
        0x00,
        0x02,
        0x00,
        0x00,
        0x00,
        0x40,
        0x01,
        0x00,
};

static uint8_t USBD_Composite_Init(USBD_HandleTypeDef *pdev,
                                   uint8_t cfgidx)
{
  uint8_t res = 0;

  pdev->pUserData = (void *)&USBD_CDC_Interface_fops_FS;
  res += USBD_CDC.Init(pdev, cfgidx);
  pCDCData = pdev->pClassData;
  /* TODO */
  pdev->pUserData = NULL;
  res += USBD_HID.Init(pdev, cfgidx);
  pHIDData = pdev->pClassData;
  return res;
}

static uint8_t USBD_Composite_DeInit(USBD_HandleTypeDef *pdev,
                                     uint8_t cfgidx)
{
  uint8_t res = 0;
  pdev->pClassData = pCDCData;
  pdev->pUserData = &USBD_CDC_Interface_fops_FS;
  res += USBD_CDC.DeInit(pdev, cfgidx);

  pdev->pClassData = pHIDData;
  /* TODO */
  pdev->pUserData = NULL;
  res += USBD_HID.DeInit(pdev, cfgidx);

  return res;
}

static uint8_t USBD_Composite_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  pdev->pClassData = pCDCData;
  pdev->pUserData = &USBD_CDC_Interface_fops_FS;
  return USBD_CDC.EP0_RxReady(pdev);
}

/**
 * @brief  USBD_Composite_Setup
 *         Handle the Composite requests
 * @param  pdev: device instance
 * @param  req: USB request
 * @retval status
 */
static uint8_t USBD_Composite_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  switch (req->bmRequest & USB_REQ_RECIPIENT_MASK)
  {
  case USB_REQ_RECIPIENT_INTERFACE:
    switch (req->wIndex)
    {
    case USBD_CDC_DATA_INTERFACE:
    case USBD_CDC_CMD_INTERFACE:
      pdev->pClassData = pCDCData;
      pdev->pUserData = &USBD_CDC_Interface_fops_FS;
      return (USBD_CDC.Setup(pdev, req));

    case USBD_HID_INTERFACE:
      pdev->pClassData = pHIDData;
      /* TODO */
      pdev->pUserData = NULL;
      return (USBD_HID.Setup(pdev, req));

    default:
      break;
    }
    break;

  case USB_REQ_RECIPIENT_ENDPOINT:
    switch (req->wIndex)
    {

    case CDC_IN_EP:
    case CDC_OUT_EP:
    case CDC_CMD_EP:
      pdev->pClassData = pCDCData;
      pdev->pUserData = &USBD_CDC_Interface_fops_FS;
      return (USBD_CDC.Setup(pdev, req));

    case HID_EPIN_ADDR:
      //         case HID_EPOUT_ADDR:
      pdev->pClassData = pHIDData;
      /* TODO */
      pdev->pUserData = NULL;
      return (USBD_HID.Setup(pdev, req));

    default:
      break;
    }
    break;
  }
  return USBD_OK;
}

/**
 * @brief  USBD_Composite_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_Composite_DataIn(USBD_HandleTypeDef *pdev,
                                     uint8_t epnum)
{
  switch (epnum)
  {
  case CDC_INDATA_NUM:
    pdev->pUserData = &USBD_CDC_Interface_fops_FS;
    pdev->pClassData = pCDCData;
    return (USBD_CDC.DataIn(pdev, epnum));
  case HID_INDATA_NUM:
    /* TODO */
    pdev->pUserData = NULL;
    pdev->pClassData = pHIDData;
    return (USBD_HID.DataIn(pdev, epnum));
  default:
    break;
  }
  return USBD_FAIL;
}

/**
 * @brief  USBD_Composite_DataOut
 *         handle data OUT Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
uint8_t USBD_Composite_DataOut(USBD_HandleTypeDef *pdev,
                               uint8_t epnum)
{
  switch (epnum)
  {
  case CDC_OUTDATA_NUM:
  case CDC_OUTCMD_NUM:
    pdev->pClassData = pCDCData;
    pdev->pUserData = &USBD_CDC_Interface_fops_FS;
    return (USBD_CDC.DataOut(pdev, epnum));

  default:
    break;
  }
  return USBD_FAIL;
}

/**
 * @brief  USBD_Composite_GetHSCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_Composite_GetFSCfgDesc(uint16_t *length)
{
  *length = sizeof(USBD_Composite_CfgFSDesc);
  return USBD_Composite_CfgFSDesc;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
uint8_t *USBD_Composite_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = sizeof(USBD_Composite_DeviceQualifierDesc);
  return USBD_Composite_DeviceQualifierDesc;
}
