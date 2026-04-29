import"./modulepreload-polyfill-B5Qt9EMX.js";const i=Object.freeze({repository:"simple_lashingrobot_ws",indexedAt:"2026-04-28 19:46:53",indexedCommit:"a2d9ad3",files:3713,symbols:29351,edges:46164,clusters:404,processes:300,source:"gitnexus status, gitnexus list, gitnexus query, gitnexus cypher"}),b=Object.freeze([{id:"tie_robot_bringup",label:"tie_robot_bringup",subtitle:"启动装配",symbols:39,x:50,y:12,tone:"amber",summary:"run.launch 汇合 api、driver_stack、algorithm_stack，是整机启动拓扑的门面。",entrypoints:["launch/run.launch","launch/driver_stack.launch","launch/algorithm_stack.launch"]},{id:"tie_robot_msgs",label:"tie_robot_msgs",subtitle:"ROS 接口账本",symbols:38,x:50,y:34,tone:"steel",summary:"集中维护 msg/srv/action，所有运行包通过它共享动作、服务和点坐标结构。",entrypoints:["msg/PointCoords.msg","srv/ProcessImage.srv","action/StartGlobalWorkTask.action"]},{id:"tie_robot_hw",label:"tie_robot_hw",subtitle:"硬件与 SDK",symbols:112,x:15,y:38,tone:"cyan",summary:"封装相机、索驱 TCP 和线性模组底层驱动，为算法与流程层提供硬件入口。",entrypoints:["src/driver/cabin_driver.cpp","src/driver/linear_module_driver.cpp","src/camera/scepter_driver.cpp"]},{id:"tie_robot_perception",label:"tie_robot_perception",subtitle:"视觉 / TF / PR-FPRG",symbols:488,x:23,y:66,tone:"green",summary:"PointAI、PR-FPRG、相机坐标点和 gripper TF 的主阵地，/perception/lashing/points_camera 保持相机原始坐标。",entrypoints:["scripts/pointai_node.py","scripts/gripper_tf_broadcaster.py","src/tie_robot_perception/pointai"]},{id:"tie_robot_process",label:"tie_robot_process",subtitle:"扫描 / 规划 / 账本",symbols:279,x:50,y:66,tone:"orange",summary:"suoquNode 编排扫描、动态分组、伪 SLAM markers、bind path 和全局执行。",entrypoints:["src/suoquNode.cpp","src/planning","data/pseudo_slam_bind_path.json"]},{id:"tie_robot_control",label:"tie_robot_control",subtitle:"线模执行",symbols:142,x:78,y:66,tone:"red",summary:"moduanNode 控制末端线性模组，执行 tie_robot_process 下发的局部绑扎点。",entrypoints:["src/moduanNode.cpp","src/moduan/linear_module_executor.cpp"]},{id:"tie_robot_web",label:"tie_robot_web",subtitle:"前端 / ROS bridge",symbols:1255,x:73,y:34,tone:"blue",summary:"新前端、Topic Layers、3D Scene 和 web bridge 都在这里，是人机交互入口。",entrypoints:["frontend/src/app/TieRobotFrontApp.js","src/web_action_bridge.cpp","scripts/workspace_picker_web_server.py"]},{id:"tie_robot_description",label:"tie_robot_description",subtitle:"URDF / RViz",symbols:7,x:86,y:18,tone:"violet",summary:"简化机器人描述与 RViz 配置，辅助 bringup 和调试可视化。",entrypoints:["URDF/model.urdf","rviz/chassis_visual.rviz"]}]),_=Object.freeze([{from:"tie_robot_bringup",to:"tie_robot_hw",label:"driver_stack",strength:"strong",description:"启动相机、索驱和线模底层节点。"},{from:"tie_robot_bringup",to:"tie_robot_perception",label:"algorithm_stack",strength:"strong",description:"启动 PointAI、稳定点 TF 和视觉算法链。"},{from:"tie_robot_bringup",to:"tie_robot_process",label:"suoquNode",strength:"strong",description:"启动流程编排节点并连接扫描/执行主链。"},{from:"tie_robot_bringup",to:"tie_robot_web",label:"api + static web",strength:"strong",description:"启动 web bridge、日志复用和静态页面服务。"},{from:"tie_robot_description",to:"tie_robot_bringup",label:"URDF / RViz",strength:"support",description:"为整机启动和可视化提供模型资源。"},{from:"tie_robot_msgs",to:"tie_robot_perception",label:"ProcessImage / PointsArray",strength:"medium",description:"视觉服务和点坐标消息使用统一接口。"},{from:"tie_robot_msgs",to:"tie_robot_process",label:"Action / Service",strength:"medium",description:"扫描、执行和流程状态通过统一消息传输。"},{from:"tie_robot_msgs",to:"tie_robot_control",label:"Move / Execute",strength:"medium",description:"线模执行消费全局消息与服务定义。"},{from:"tie_robot_msgs",to:"tie_robot_web",label:"rosbridge types",strength:"medium",description:"前端和 web bridge 使用相同 ROS 类型。"},{from:"tie_robot_hw",to:"tie_robot_perception",label:"Scepter frames",strength:"medium",description:"相机驱动和世界点进入视觉识别链。"},{from:"tie_robot_hw",to:"tie_robot_process",label:"cabin transport",strength:"strong",description:"索驱 TCP 状态和运动控制进入 suoqu 流程层。"},{from:"tie_robot_perception",to:"tie_robot_process",label:"/perception/lashing/recognize_once",strength:"strong",description:"扫描和执行微调请求视觉，回收 map 下的绑扎点。"},{from:"tie_robot_process",to:"tie_robot_control",label:"ExecuteBindPoints",strength:"strong",description:"流程层按账本生成局部点并下发给末端执行。"},{from:"tie_robot_web",to:"tie_robot_process",label:"scan / global work",strength:"strong",description:"前端发起伪 SLAM 扫描、全局执行和账本测试。"},{from:"tie_robot_web",to:"tie_robot_perception",label:"IR / S2 / point cloud",strength:"strong",description:"前端消费相机、S2 覆盖层、点云与绑扎点话题。"},{from:"tie_robot_web",to:"tie_robot_control",label:"moduan debug",strength:"medium",description:"控制台保留线模调试与状态查看入口。"},{from:"tie_robot_process",to:"tie_robot_web",label:"markers / bind path",strength:"strong",description:"流程层发布规划 markers，静态 API 暴露 bind path 给 3D Scene。"}]),g=Object.freeze([{name:"扫描建图链",route:["tie_robot_web","tie_robot_process","tie_robot_perception","tie_robot_process","tie_robot_web"],detail:"前端启动扫描，流程层移动索驱并请求 PR-FPRG，视觉输出 map 点，流程层写 pseudo_slam_points/bind_path，再回到 3D Scene。"},{name:"真实执行链",route:["tie_robot_web","tie_robot_process","tie_robot_control"],detail:"前端触发全局执行，process 按 bind path 规划区域，control 执行局部 2x2 绑扎点并回写记忆。"},{name:"可视化链",route:["tie_robot_perception","tie_robot_process","tie_robot_web"],detail:"TF、相机坐标系 /perception/lashing/points_camera、pseudo_slam_markers 和点云共同进入新前端，形成 3D Scene 与 Topic Layers。"}]),p=document.getElementById("projectGraphApp"),a=new Map(b.map(e=>[e.id,e]));function c(e){return new Intl.NumberFormat("zh-CN").format(e)}function y(e,o){const r=!o||e.from===o||e.to===o;return["graph-edge",`is-${e.strength}`,r?"is-active":"is-muted"].join(" ")}function h(){return[["Files",i.files],["Symbols",i.symbols],["Edges",i.edges],["Processes",i.processes]].map(([o,r],t)=>`
    <article class="summary-card" style="--delay: ${t*70}ms">
      <span>${o}</span>
      <strong>${c(r)}</strong>
    </article>
  `).join("")}function f(e=null){const o=_.map(t=>{const s=a.get(t.from),n=a.get(t.to);if(!s||!n)return"";const d=(s.x+n.x)/2,m=(s.y+n.y)/2,u=t.strength==="support"?-7:0;return`
      <path
        class="${y(t,e)}"
        data-edge-from="${t.from}"
        data-edge-to="${t.to}"
        d="M ${s.x} ${s.y} Q ${d} ${m+u} ${n.x} ${n.y}"
      />
    `}).join(""),r=b.map(t=>{const s=!e||e===t.id;return`
      <button
        class="graph-node is-${t.tone} ${s?"is-active":"is-muted"}"
        data-node-id="${t.id}"
        style="left: ${t.x}%; top: ${t.y}%"
        type="button"
        aria-label="查看 ${t.label} 关系"
      >
        <span class="node-orbit"></span>
        <strong>${t.label}</strong>
        <small>${t.subtitle}</small>
        <em>${t.symbols} symbols</em>
      </button>
    `}).join("");return`
    <div class="graph-board">
      <svg class="graph-lines" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
        ${o}
      </svg>
      ${r}
    </div>
  `}function $(e="tie_robot_process"){const o=a.get(e)||a.get("tie_robot_process"),r=_.filter(t=>t.from===o.id||t.to===o.id);return`
    <section class="detail-card">
      <p class="eyebrow">Selected Package</p>
      <h2>${o.label}</h2>
      <p>${o.summary}</p>
      <div class="entry-list">
        ${o.entrypoints.map(t=>`<span>${t}</span>`).join("")}
      </div>
    </section>
    <section class="detail-card detail-card-edges">
      <p class="eyebrow">Relations</p>
      <h2>关系强度</h2>
      ${r.map(t=>{var s,n;return`
        <article>
          <strong>${((s=a.get(t.from))==null?void 0:s.label)||t.from} -> ${((n=a.get(t.to))==null?void 0:n.label)||t.to}</strong>
          <span>${t.label}</span>
          <p>${t.description}</p>
        </article>
      `}).join("")}
    </section>
  `}function w(){return g.map((e,o)=>`
    <article class="flow-card" style="--delay: ${160+o*90}ms">
      <span class="flow-index">0${o+1}</span>
      <h3>${e.name}</h3>
      <div class="flow-route">
        ${e.route.map(r=>{var t;return`<span>${((t=a.get(r))==null?void 0:t.subtitle)||r}</span>`}).join("<b></b>")}
      </div>
      <p>${e.detail}</p>
    </article>
  `).join("")}function l(e="tie_robot_process"){p.innerHTML=`
    <main class="project-graph-page">
      <section class="hero">
        <div>
          <p class="eyebrow">GitNexus Knowledge Graph</p>
          <h1>绑扎机器人工程关系图</h1>
          <p class="hero-copy">
            由 GitNexus 索引当前仓库后整理：${c(i.files)} 个文件、
            ${c(i.symbols)} 个符号、${c(i.edges)} 条关系边。
            页面展示 ROS 包之间的主依赖、运行链路和调试入口。
          </p>
        </div>
        <aside class="index-stamp">
          <span>Indexed</span>
          <strong>${i.indexedAt}</strong>
          <small>${i.indexedCommit} / ${i.source}</small>
        </aside>
      </section>

      <section class="summary-grid">
        ${h()}
      </section>

      <section class="graph-shell">
        <div class="graph-header">
          <div>
            <p class="eyebrow">Package Topology</p>
            <h2>从启动到视觉、规划、执行、前端的主链</h2>
          </div>
          <button class="reset-button" type="button" data-reset-graph>显示全图</button>
        </div>
        ${f(e)}
      </section>

      <section class="detail-grid">
        ${$(e)}
      </section>

      <section class="flow-grid">
        ${w()}
      </section>
    </main>
  `,v()}function v(){var e;p.querySelectorAll("[data-node-id]").forEach(o=>{o.addEventListener("click",()=>{l(o.getAttribute("data-node-id"))})}),(e=p.querySelector("[data-reset-graph]"))==null||e.addEventListener("click",()=>l(null))}l();
