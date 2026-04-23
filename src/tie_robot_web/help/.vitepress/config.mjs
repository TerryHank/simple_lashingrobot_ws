export default {
  title: "Tie Robot Help",
  description: "simple_lashingrobot_ws 工程结构与主链说明",
  lang: "zh-CN",
  base: "/help/",
  cleanUrls: true,
  themeConfig: {
    nav: [
      { text: "总览", link: "/" },
      { text: "工程说明", link: "/guide/overview" },
      { text: "视觉原理", link: "/guide/visual-principles" },
      { text: "相机 SDK", link: "/camera-sdk/index" },
      { text: "文件树", link: "/reference/file-tree" }
    ],
    sidebar: [
      {
        text: "工程说明",
        items: [
          { text: "工程总览", link: "/guide/overview" },
          { text: "运行主链", link: "/guide/runtime-flows" },
          { text: "视觉原理", link: "/guide/visual-principles" },
          { text: "开发入口", link: "/guide/dev-entrypoints" }
        ]
      },
      {
        text: "相机 SDK",
        items: [
          { text: "相机 SDK 总览", link: "/camera-sdk/index" },
          { text: "原厂文档入口", link: "/camera-sdk/vendor-vzense/README" },
          { text: "中文入口", link: "/camera-sdk/vendor-vzense/zh-cn/README" },
          { text: "中文 Quickstart", link: "/camera-sdk/vendor-vzense/zh-cn/Quickstart/Quickstart" },
          { text: "中文 SDK 概述", link: "/camera-sdk/vendor-vzense/zh-cn/ScepterSDK/Overview" },
          { text: "中文 BaseSDK", link: "/camera-sdk/vendor-vzense/zh-cn/ScepterSDK/BaseSDK" },
          { text: "中文 ROS 插件", link: "/camera-sdk/vendor-vzense/zh-cn/ScepterSDK/3rd-Party-Plugin/ROS" },
          { text: "中文 GUI FAQ", link: "/camera-sdk/vendor-vzense/zh-cn/ScepterGUITool/FAQ" },
          { text: "English Entry", link: "/camera-sdk/vendor-vzense/en/README" },
          { text: "English BaseSDK", link: "/camera-sdk/vendor-vzense/en/ScepterSDK/BaseSDK" }
        ]
      },
      {
        text: "参考",
        items: [
          { text: "文件结构树", link: "/reference/file-tree" },
          { text: "重构映射", link: "/reference/refactor-map" }
        ]
      }
    ]
  }
};
