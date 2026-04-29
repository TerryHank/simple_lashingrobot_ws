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
      { text: "系统设计", link: "/guide/system-design" },
      { text: "ROS Graph", link: "/guide/ros-graph" },
      { text: "视觉原理", link: "/guide/visual-principles" },
      { text: "PR-FPRG 流程", link: "/guide/pr-fprg-workflow" },
      {
        text: "对外接入",
        items: [
          { text: "动态 API 网关", link: "/guide/dynamic-api-gateway" },
          { text: "GB28181 视频接入", link: "/guide/gb28181-video-gateway" }
        ]
      },
      { text: "相机 SDK", link: "/camera-sdk/index" },
      { text: "GitNexus 图谱", link: "/reference/gitnexus-graph" },
      { text: "文件树", link: "/reference/file-tree" }
    ],
    sidebar: [
      {
        text: "工程说明",
        items: [
          { text: "工程总览", link: "/guide/overview" },
          { text: "工程设计与架构图", link: "/guide/system-design" },
          { text: "ROS Graph", link: "/guide/ros-graph" },
          { text: "运行主链", link: "/guide/runtime-flows" },
          { text: "视觉原理", link: "/guide/visual-principles" },
          { text: "PR-FPRG 流程详解", link: "/guide/pr-fprg-workflow" },
          { text: "开发入口", link: "/guide/dev-entrypoints" }
        ]
      },
      {
        text: "对外接入",
        items: [
          { text: "ROS 动态 API 网关", link: "/guide/dynamic-api-gateway" },
          { text: "GB28181 视频接入", link: "/guide/gb28181-video-gateway" }
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
        text: "GitNexus 图谱",
        items: [
          { text: "本地 WebUI", link: "/reference/gitnexus-graph" }
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
