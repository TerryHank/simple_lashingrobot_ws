# GitNexus 本地 WebUI 图谱

<script setup>
import { computed, onMounted, ref } from "vue";

const browserHost = ref("");
const projectName = "simple_lashingrobot_ws";

onMounted(() => {
  browserHost.value = window.location.hostname || "127.0.0.1";
});

const gitnexusHost = computed(() => browserHost.value || "127.0.0.1");
const webuiOrigin = computed(() => `http://${gitnexusHost.value}:5173`);
const bridgeOrigin = computed(() => `http://${gitnexusHost.value}:4747`);
const bridgeReposUrl = computed(() => `${bridgeOrigin.value}/api/repos`);
const gitnexusWebuiUrl = computed(() => {
  const query = new URLSearchParams({
    server: bridgeOrigin.value,
    project: projectName,
  });
  return `${webuiOrigin.value}/?${query.toString()}`;
});
</script>

本页使用 GitNexus 仓库里的 `gitnexus-web` 本地 WebUI 显示当前 `simple_lashingrobot_ws` 的知识图谱，不再跳转到外部托管版页面。

当前本地 GitNexus 索引已经解析出：

- `3713 files`
- `29351 symbols`
- `46164 edges`
- `404 clusters`
- `300 processes`

## 使用方式

在工作区根目录启动本地 WebUI。脚本会在本机缓存 GitNexus 源码，启动 `gitnexus serve`，再启动 GitNexus WebUI 的 Vite 开发服务：

```bash
src/tie_robot_web/tools/run_gitnexus_local_webui.py
```

如果要手动启动，分别运行：

```bash
gitnexus serve --host 0.0.0.0 --port 4747
cd ~/.cache/tie_robot_web/GitNexus/gitnexus-web
npm install
npm run dev -- --host 0.0.0.0 --port 5173
```

下面的 iframe 会使用当前浏览器访问帮助站时的主机名生成 WebUI 地址。也就是说，在本机打开时走本机地址，在局域网里通过 `192.168.x.x:8080` 打开时，会自动连接同一台机器的 `5173` 和 `4747` 端口。

<ClientOnly>
<div class="gitnexus-actions">
  <a class="gitnexus-open-link" :href="gitnexusWebuiUrl" target="_blank" rel="noreferrer">
    在新标签打开 GitNexus WebUI
  </a>
  <code>{{ webuiOrigin }}</code>
  <code>{{ bridgeReposUrl }}</code>
</div>

<iframe
  class="gitnexus-webui-frame"
  title="GitNexus WebUI - simple_lashingrobot_ws"
  :src="gitnexusWebuiUrl"
  loading="lazy"
  referrerpolicy="no-referrer"
></iframe>
</ClientOnly>

## 说明

- WebUI 页面本身来自本机运行的 GitNexus `gitnexus-web`，关系图数据来自本机 `gitnexus serve` 暴露的本地索引。
- 如果 iframe 里显示无法连接，先确认脚本正在运行，并确认局域网客户端可以访问当前主机的 `5173` 和 `4747` 端口。
- 如果有防火墙，放行 `8080`、`5173`、`4747` 三个端口；`8080` 是帮助站，`5173` 是 GitNexus WebUI，`4747` 是 GitNexus API。
- 如果仓库结构有较大改动，先执行 `gitnexus analyze` 更新索引，再刷新本页。

<style>
.gitnexus-actions {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: 12px;
  margin: 16px 0;
}

.vp-doc .gitnexus-open-link {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  min-height: 38px;
  padding: 0 16px;
  border-radius: 999px;
  border: 1px solid rgba(151, 189, 255, 0.38);
  background: linear-gradient(135deg, #d7e8ff, #a8ead7);
  color: #071018 !important;
  font-weight: 800;
  letter-spacing: 0.01em;
  text-decoration: none;
  box-shadow: 0 10px 30px rgba(87, 171, 190, 0.18);
}

.vp-doc .gitnexus-open-link:hover {
  color: #071018 !important;
  border-color: rgba(190, 221, 255, 0.62);
  background: linear-gradient(135deg, #eef6ff, #c1f2e4);
}

.gitnexus-webui-frame {
  width: 100%;
  height: min(76vh, 820px);
  min-height: 620px;
  border: 1px solid rgba(125, 141, 164, 0.28);
  border-radius: 18px;
  background: #060913;
  box-shadow: 0 18px 50px rgba(0, 0, 0, 0.28);
}
</style>
