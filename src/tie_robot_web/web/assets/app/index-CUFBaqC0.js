(function(){const e=document.createElement("link").relList;if(e&&e.supports&&e.supports("modulepreload"))return;for(const r of document.querySelectorAll('link[rel="modulepreload"]'))n(r);new MutationObserver(r=>{for(const s of r)if(s.type==="childList")for(const o of s.addedNodes)o.tagName==="LINK"&&o.rel==="modulepreload"&&n(o)}).observe(document,{childList:!0,subtree:!0});function t(r){const s={};return r.integrity&&(s.integrity=r.integrity),r.referrerPolicy&&(s.referrerPolicy=r.referrerPolicy),r.crossOrigin==="use-credentials"?s.credentials="include":r.crossOrigin==="anonymous"?s.credentials="omit":s.credentials="same-origin",s}function n(r){if(r.ep)return;r.ep=!0;const s=t(r);fetch(r.href,s)}})();const Ia={globalX:0,globalY:0,globalZ:10,speed:300,stepX:370,stepY:320,startX:0,startY:0,startZ:0,fixedX:0,fixedY:0,fixedZ:0,fixedTheta:0,fixedSpeed:30},ua=[{id:1,name:"启动机器人",topic:"/web/cabin/start",type:"std_msgs/Float32",group:"流程控制"},{id:2,name:"规划作业路径",topic:"/web/cabin/plan_path",type:"geometry_msgs/Pose",group:"流程控制"},{id:3,name:"清除作业路径",topic:"/web/cabin/clear_path",type:"std_msgs/Float32",group:"流程控制"},{id:4,name:"开始全局作业",topic:"/web/cabin/start_global_work",type:"std_msgs/Float32",group:"流程控制"},{id:5,name:"重启机器人",topic:"/web/cabin/restart",type:"std_msgs/Float32",group:"流程控制"},{id:6,name:"末端运动调试",topic:"/web/moduan/moduan_move_debug",type:"geometry_msgs/Pose",group:"调试控制"},{id:7,name:"索驱运动调试",topic:"/web/cabin/cabin_move_debug",type:"geometry_msgs/Pose",group:"调试控制"},{id:8,name:"视觉识别调试",topic:"/web/pointAI/process_image",type:"std_msgs/Float32",group:"视觉调试"},{id:9,name:"定点绑扎调试",topic:"/web/moduan/single_bind",type:"std_msgs/Float32",group:"调试控制"},{id:10,name:"暂停作业",topic:"/web/moduan/interrupt_stop",type:"std_msgs/Float32",group:"末端控制"},{id:11,name:"开启绑扎",topic:"/web/moduan/enb_las",type:"std_msgs/Float32",group:"末端控制"},{id:12,name:"开启(关闭)跳绑",topic:"/web/moduan/send_odd_points",type:"std_msgs/Bool",group:"末端控制"},{id:13,name:"恢复作业",topic:"/web/moduan/hand_sovle_warn",type:"std_msgs/Float32",group:"末端控制"},{id:14,name:"开启(关闭)灯光",topic:"/web/moduan/light",type:"std_msgs/Bool",group:"末端控制"},{id:15,name:"末端回零",topic:"/web/moduan/moduan_move_zero",type:"std_msgs/Float32",group:"末端控制"},{id:16,name:"关闭绑扎",topic:"/web/moduan/enb_las",type:"std_msgs/Float32",group:"末端控制"},{id:17,name:"关闭机器人",topic:"/web/cabin/shutdown",type:"std_msgs/Float32",group:"流程控制"},{id:18,name:"急停作业",topic:"/web/moduan/forced_stop",type:"std_msgs/Float32",group:"末端控制"},{id:19,name:"设置Z固定高度",topic:"/web/pointAI/set_height_threshold",type:"std_msgs/Float32",group:"视觉调试"},{id:20,name:"保存作业路径",topic:"/web/cabin/save_path",type:"std_msgs/Float32",group:"流程控制"},{id:21,name:"保存绑扎数据",topic:"/web/moduan/save_binding_data",type:"std_msgs/Float32",group:"末端控制"},{id:22,name:"修正视觉偏差",topic:"/web/pointAI/set_offset",type:"geometry_msgs/Pose",group:"视觉调试"},{id:23,name:"设置索驱速度",topic:"/web/cabin/set_cabin_speed",type:"std_msgs/Float32",group:"流程控制"},{id:24,name:"设置末端速度",topic:"/web/moduan/set_moduan_speed",type:"std_msgs/Float32",group:"末端控制"}],Mu=[{id:"submitQuad",label:`提交四边形
触发 S2`,tone:"green"},{id:"runSavedS2",label:`直接识别
绑扎点`,tone:"green"},{id:"scanPlan",label:`固定扫描
规划`,tone:"green"},{id:"startExecution",label:`开始
执行层`,tone:"green"},{id:"startExecutionKeepMemory",label:`保留记忆
开始`,tone:"amber"},{id:"runBindPathTest",label:`账本
测试`,tone:"amber"}],dc={pauseResume:{id:"pauseResume",group:"末端控制",stateKey:"paused",initialValue:!1,inactiveLabel:"暂停作业",activeLabel:"恢复作业",inactiveTone:"amber",activeTone:"green",activateCommandId:10,deactivateCommandId:13},lashingEnabled:{id:"lashingEnabled",group:"末端控制",stateKey:"enabled",initialValue:!0,inactiveLabel:"开启绑扎",activeLabel:"关闭绑扎",inactiveTone:"green",activeTone:"red",activateCommandId:11,deactivateCommandId:16},jumpBindEnabled:{id:"jumpBindEnabled",group:"末端控制",stateKey:"enabled",initialValue:!1,inactiveLabel:"开启跳绑",activeLabel:"关闭跳绑",inactiveTone:"blue",activeTone:"amber",commandId:12,messageType:"std_msgs/Bool"},lightEnabled:{id:"lightEnabled",group:"末端控制",stateKey:"enabled",initialValue:!1,inactiveLabel:"开启灯光",activeLabel:"关闭灯光",inactiveTone:"blue",activeTone:"amber",commandId:14,messageType:"std_msgs/Bool"}},bu=[{title:"流程控制",items:[1,2,3,4,5,17,20,23]},{title:"调试控制",items:[7,6,8,9]},{title:"末端控制",items:["pauseResume","lashingEnabled","jumpBindEnabled","lightEnabled",15,18,21,24]},{title:"视觉调试",items:[19,22]}],Eu={1:"green",2:"green",3:"red",4:"green",5:"amber",6:"amber",7:"amber",8:"amber",9:"amber",15:"green",17:"red",18:"red",19:"blue",20:"blue",21:"blue",22:"blue",23:"blue",24:"blue"};function fc(i){return Eu[i]||"amber"}function cn(i){return dc[i]||null}function pc(i){const e=cn(i);if(!e)return null;const t=!!e.initialValue;return{value:t,label:t?e.activeLabel:e.inactiveLabel,tone:t?e.activeTone:e.inactiveTone}}function hr(){return Object.keys(dc).reduce((i,e)=>(i[e]=pc(e),i),{})}function Tu(){return bu.map(i=>({title:i.title,controls:i.items.map(e=>{if(typeof e=="string"){const n=cn(e);if(!n)return null;const r=pc(e);return{kind:"toggle",id:n.id,label:(r==null?void 0:r.label)||n.inactiveLabel,tone:(r==null?void 0:r.tone)||n.inactiveTone,active:!!(r!=null&&r.value)}}const t=ua.find(n=>n.id===e);return t?{kind:"command",id:String(t.id),commandId:t.id,label:t.name,tone:fc(t.id)}:null}).filter(Boolean)}))}var mc=typeof globalThis<"u"?globalThis:typeof window<"u"?window:typeof global<"u"?global:typeof self<"u"?self:{};function wu(i){return i&&i.__esModule&&Object.prototype.hasOwnProperty.call(i,"default")?i.default:i}/*
object-assign
(c) Sindre Sorhus
@license MIT
*/var Na=Object.getOwnPropertySymbols,Au=Object.prototype.hasOwnProperty,Cu=Object.prototype.propertyIsEnumerable;function Ru(i){if(i==null)throw new TypeError("Object.assign cannot be called with null or undefined");return Object(i)}function Pu(){try{if(!Object.assign)return!1;var i=new String("abc");if(i[5]="de",Object.getOwnPropertyNames(i)[0]==="5")return!1;for(var e={},t=0;t<10;t++)e["_"+String.fromCharCode(t)]=t;var n=Object.getOwnPropertyNames(e).map(function(s){return e[s]});if(n.join("")!=="0123456789")return!1;var r={};return"abcdefghijklmnopqrst".split("").forEach(function(s){r[s]=s}),Object.keys(Object.assign({},r)).join("")==="abcdefghijklmnopqrst"}catch{return!1}}var Nn=Pu()?Object.assign:function(i,e){for(var t,n=Ru(i),r,s=1;s<arguments.length;s++){t=Object(arguments[s]);for(var o in t)Au.call(t,o)&&(n[o]=t[o]);if(Na){r=Na(t);for(var a=0;a<r.length;a++)Cu.call(t,r[a])&&(n[r[a]]=t[r[a]])}}return n},Dr={exports:{}},Ir,Oa;function ya(){return Oa||(Oa=1,Ir=function(i,e,t){e.forEach(function(n){var r=t[n];i.prototype[n]=function(s){return s.ros=this,new r(s)}})}),Ir}var Nr,Fa;function gc(){return Fa||(Fa=1,Nr=typeof window<"u"?window.WebSocket:WebSocket),Nr}var Or,Ba;function Lu(){if(Ba)return Or;Ba=1;var i=arguments[3],e=arguments[4],t=arguments[5],n=JSON.stringify;return Or=function(r,s){for(var o,a=Object.keys(t),l=0,c=a.length;l<c;l++){var u=a[l],h=t[u].exports;if(h===r||h&&h.default===r){o=u;break}}if(!o){o=Math.floor(Math.pow(16,8)*Math.random()).toString(16);for(var d={},l=0,c=a.length;l<c;l++){var u=a[l];d[u]=u}e[o]=["function(require,module,exports){"+r+"(self); }",d]}var f=Math.floor(Math.pow(16,8)*Math.random()).toString(16),g={};g[o]=o,e[f]=["function(require,module,exports){var f = require("+n(o)+");(f.default ? f.default : f)(self);}",g];var x={};m(f);function m(D){x[D]=!0;for(var P in e[D][1]){var X=e[D][1][P];x[X]||m(X)}}var p="("+i+")({"+Object.keys(x).map(function(D){return n(D)+":["+e[D][0]+","+n(e[D][1])+"]"}).join(",")+"},{},["+n(f)+"])",T=window.URL||window.webkitURL||window.mozURL||window.msURL,S=new Blob([p],{type:"text/javascript"});if(s&&s.bare)return S;var A=T.createObjectURL(S),B=new Worker(A);return B.objectURL=A,B},Or}var Fr={exports:{}},ka;function Uu(){return ka||(ka=1,function(i){function e(c){var u={};function h(f){if(u[f])return u[f].exports;var g=u[f]={i:f,l:!1,exports:{}};return c[f].call(g.exports,g,g.exports,h),g.l=!0,g.exports}h.m=c,h.c=u,h.i=function(f){return f},h.d=function(f,g,x){h.o(f,g)||Object.defineProperty(f,g,{configurable:!1,enumerable:!0,get:x})},h.r=function(f){Object.defineProperty(f,"__esModule",{value:!0})},h.n=function(f){var g=f&&f.__esModule?function(){return f.default}:function(){return f};return h.d(g,"a",g),g},h.o=function(f,g){return Object.prototype.hasOwnProperty.call(f,g)},h.p="/",h.oe=function(f){throw console.error(f),f};var d=h(h.s=ENTRY_MODULE);return d.default||d}var t="[\\.|\\-|\\+|\\w|/|@]+",n="\\(\\s*(/\\*.*?\\*/)?\\s*.*?("+t+").*?\\)";function r(c){return(c+"").replace(/[.?*+^$[\]\\(){}|-]/g,"\\$&")}function s(c){return!isNaN(1*c)}function o(c,u,h){var d={};d[h]=[];var f=u.toString(),g=f.match(/^function\s?\w*\(\w+,\s*\w+,\s*(\w+)\)/);if(!g)return d;for(var x=g[1],m=new RegExp("(\\\\n|\\W)"+r(x)+n,"g"),p;p=m.exec(f);)p[3]!=="dll-reference"&&d[h].push(p[3]);for(m=new RegExp("\\("+r(x)+'\\("(dll-reference\\s('+t+'))"\\)\\)'+n,"g");p=m.exec(f);)c[p[2]]||(d[h].push(p[1]),c[p[2]]=__webpack_require__(p[1]).m),d[p[2]]=d[p[2]]||[],d[p[2]].push(p[4]);for(var T=Object.keys(d),S=0;S<T.length;S++)for(var A=0;A<d[T[S]].length;A++)s(d[T[S]][A])&&(d[T[S]][A]=1*d[T[S]][A]);return d}function a(c){var u=Object.keys(c);return u.reduce(function(h,d){return h||c[d].length>0},!1)}function l(c,u){for(var h={main:[u]},d={main:[]},f={main:{}};a(h);)for(var g=Object.keys(h),x=0;x<g.length;x++){var m=g[x],p=h[m],T=p.pop();if(f[m]=f[m]||{},!(f[m][T]||!c[m][T])){f[m][T]=!0,d[m]=d[m]||[],d[m].push(T);for(var S=o(c,c[m][T],m),A=Object.keys(S),B=0;B<A.length;B++)h[A[B]]=h[A[B]]||[],h[A[B]]=h[A[B]].concat(S[A[B]])}}return d}i.exports=function(c,u){u=u||{};var h={main:__webpack_modules__},d=u.all?{main:Object.keys(h.main)}:l(h,c),f="";Object.keys(d).filter(function(T){return T!=="main"}).forEach(function(T){for(var S=0;d[T][S];)S++;d[T].push(S),h[T][S]="(function(module, exports, __webpack_require__) { module.exports = __webpack_require__; })",f=f+"var "+T+" = ("+e.toString().replace("ENTRY_MODULE",JSON.stringify(S))+")({"+d[T].map(function(A){return""+JSON.stringify(A)+": "+h[T][A].toString()}).join(",")+`});
`}),f=f+"new (("+e.toString().replace("ENTRY_MODULE",JSON.stringify(c))+")({"+d.main.map(function(T){return""+JSON.stringify(T)+": "+h.main[T].toString()}).join(",")+"}))(self);";var g=new window.Blob([f],{type:"text/javascript"});if(u.bare)return g;var x=window.URL||window.webkitURL||window.mozURL||window.msURL,m=x.createObjectURL(g),p=new window.Worker(m);return p.objectURL=m,p}}(Fr)),Fr.exports}var Br,za;function Du(){if(za)return Br;za=1;var i=i||gc();return Br=function(e){var t=null;function n(s){var o=s.data;o instanceof ArrayBuffer?e.postMessage(o,[o]):e.postMessage(o)}function r(s){e.postMessage({type:s.type})}e.addEventListener("message",function(s){var o=s.data;if(typeof o=="string")t.send(o);else if(o.hasOwnProperty("close"))t.close(),t=null;else if(o.hasOwnProperty("uri")){var a=o.uri;t=new i(a),t.binaryType="arraybuffer",t.onmessage=n,t.onclose=r,t.onopen=r,t.onerror=r}else throw"Unknown message to WorkerSocket"})},Br}var kr,Va;function Iu(){if(Va)return kr;Va=1;try{var i=Lu()}catch{var i=Uu()}var e=Du();function t(n){this.socket_=i(e),this.socket_.addEventListener("message",this.handleWorkerMessage_.bind(this)),this.socket_.postMessage({uri:n})}return t.prototype.handleWorkerMessage_=function(n){var r=n.data;if(r instanceof ArrayBuffer||typeof r=="string")this.onmessage(n);else{var s=r.type;if(s==="close")this.onclose(null);else if(s==="open")this.onopen(null);else if(s==="error")this.onerror(null);else throw"Unknown message from workersocket"}},t.prototype.send=function(n){this.socket_.postMessage(n)},t.prototype.close=function(){this.socket_.postMessage({close:!0})},kr=t,kr}var zr,Ga;function Nu(){return Ga||(Ga=1,zr=function(){return document.createElement("canvas")}),zr}var Vr,Ha;function Ou(){if(Ha)return Vr;Ha=1;var i=Nu(),e=i.Image||window.Image;function t(n,r){var s=new e;s.onload=function(){var o=new i,a=o.getContext("2d");o.width=s.width,o.height=s.height,a.imageSmoothingEnabled=!1,a.webkitImageSmoothingEnabled=!1,a.mozImageSmoothingEnabled=!1,a.drawImage(s,0,0);for(var l=a.getImageData(0,0,s.width,s.height).data,c="",u=0;u<l.length;u+=4)c+=String.fromCharCode(l[u],l[u+1],l[u+2]);r(JSON.parse(c))},s.src="data:image/png;base64,"+n}return Vr=t,Vr}var Gr={exports:{}},Wa;function Fu(){return Wa||(Wa=1,function(i){(function(e,t){var n=Math.pow(2,-24),r=Math.pow(2,32),s=Math.pow(2,53);function o(c){var u=new ArrayBuffer(256),h=new DataView(u),d,f=0;function g(C){for(var H=u.byteLength,O=f+C;H<O;)H*=2;if(H!==u.byteLength){var $=h;u=new ArrayBuffer(H),h=new DataView(u);for(var V=f+3>>2,Y=0;Y<V;++Y)h.setUint32(Y*4,$.getUint32(Y*4))}return d=C,h}function x(){f+=d}function m(C){x(g(8).setFloat64(f,C))}function p(C){x(g(1).setUint8(f,C))}function T(C){for(var H=g(C.length),O=0;O<C.length;++O)H.setUint8(f+O,C[O]);x()}function S(C){x(g(2).setUint16(f,C))}function A(C){x(g(4).setUint32(f,C))}function B(C){var H=C%r,O=(C-H)/r,$=g(8);$.setUint32(f,O),$.setUint32(f+4,H),x()}function D(C,H){H<24?p(C<<5|H):H<256?(p(C<<5|24),p(H)):H<65536?(p(C<<5|25),S(H)):H<4294967296?(p(C<<5|26),A(H)):(p(C<<5|27),B(H))}function P(C){var H;if(C===!1)return p(244);if(C===!0)return p(245);if(C===null)return p(246);if(C===t)return p(247);switch(typeof C){case"number":if(Math.floor(C)===C){if(0<=C&&C<=s)return D(0,C);if(-s<=C&&C<0)return D(1,-(C+1))}return p(251),m(C);case"string":var O=[];for(H=0;H<C.length;++H){var $=C.charCodeAt(H);$<128?O.push($):$<2048?(O.push(192|$>>6),O.push(128|$&63)):$<55296?(O.push(224|$>>12),O.push(128|$>>6&63),O.push(128|$&63)):($=($&1023)<<10,$|=C.charCodeAt(++H)&1023,$+=65536,O.push(240|$>>18),O.push(128|$>>12&63),O.push(128|$>>6&63),O.push(128|$&63))}return D(3,O.length),T(O);default:var V;if(Array.isArray(C))for(V=C.length,D(4,V),H=0;H<V;++H)P(C[H]);else if(C instanceof Uint8Array)D(2,C.length),T(C);else{var Y=Object.keys(C);for(V=Y.length,D(5,V),H=0;H<V;++H){var te=Y[H];P(te),P(C[te])}}}}if(P(c),"slice"in u)return u.slice(0,f);for(var X=new ArrayBuffer(f),E=new DataView(X),M=0;M<f;++M)E.setUint8(M,h.getUint8(M));return X}function a(c,u,h){var d=new DataView(c),f=0;typeof u!="function"&&(u=function(O){return O}),typeof h!="function"&&(h=function(){return t});function g(O,$){return f+=$,O}function x(O){return g(new Uint8Array(c,f,O),O)}function m(){var O=new ArrayBuffer(4),$=new DataView(O),V=A(),Y=V&32768,te=V&31744,v=V&1023;if(te===31744)te=261120;else if(te!==0)te+=114688;else if(v!==0)return v*n;return $.setUint32(0,Y<<16|te<<13|v<<13),$.getFloat32(0)}function p(){return g(d.getFloat32(f),4)}function T(){return g(d.getFloat64(f),8)}function S(){return g(d.getUint8(f),1)}function A(){return g(d.getUint16(f),2)}function B(){return g(d.getUint32(f),4)}function D(){return B()*r+B()}function P(){return d.getUint8(f)!==255?!1:(f+=1,!0)}function X(O){if(O<24)return O;if(O===24)return S();if(O===25)return A();if(O===26)return B();if(O===27)return D();if(O===31)return-1;throw"Invalid length encoding"}function E(O){var $=S();if($===255)return-1;var V=X($&31);if(V<0||$>>5!==O)throw"Invalid indefinite length element";return V}function M(O,$){for(var V=0;V<$;++V){var Y=S();Y&128&&(Y<224?(Y=(Y&31)<<6|S()&63,$-=1):Y<240?(Y=(Y&15)<<12|(S()&63)<<6|S()&63,$-=2):(Y=(Y&15)<<18|(S()&63)<<12|(S()&63)<<6|S()&63,$-=3)),Y<65536?O.push(Y):(Y-=65536,O.push(55296|Y>>10),O.push(56320|Y&1023))}}function C(){var O=S(),$=O>>5,V=O&31,Y,te;if($===7)switch(V){case 25:return m();case 26:return p();case 27:return T()}if(te=X(V),te<0&&($<2||6<$))throw"Invalid length";switch($){case 0:return te;case 1:return-1-te;case 2:if(te<0){for(var v=[],w=0;(te=E($))>=0;)w+=te,v.push(x(te));var U=new Uint8Array(w),N=0;for(Y=0;Y<v.length;++Y)U.set(v[Y],N),N+=v[Y].length;return U}return x(te);case 3:var G=[];if(te<0)for(;(te=E($))>=0;)M(G,te);else M(G,te);return String.fromCharCode.apply(null,G);case 4:var K;if(te<0)for(K=[];!P();)K.push(C());else for(K=new Array(te),Y=0;Y<te;++Y)K[Y]=C();return K;case 5:var I={};for(Y=0;Y<te||te<0&&!P();++Y){var k=C();I[k]=C()}return I;case 6:return u(C(),te);case 7:switch(te){case 20:return!1;case 21:return!0;case 22:return null;case 23:return t;default:return h(te)}}}var H=C();if(f!==c.byteLength)throw"Remaining bytes";return H}var l={encode:o,decode:a};i.exports?i.exports=l:e.CBOR||(e.CBOR=l)})(mc)}(Gr)),Gr.exports}var Hr={exports:{}},Xa;function Bu(){return Xa||(Xa=1,function(i){var e=Math.pow(2,32),t=!1;function n(){t||(t=!0,console.warn("CBOR 64-bit integer array values may lose precision. No further warnings."))}function r(u){n();for(var h=u.byteLength,d=u.byteOffset,f=h/8,g=u.buffer.slice(d,d+h),x=new Uint32Array(g),m=new Array(f),p=0;p<f;p++){var T=p*2,S=x[T],A=x[T+1];m[p]=S+e*A}return m}function s(u){n();for(var h=u.byteLength,d=u.byteOffset,f=h/8,g=u.buffer.slice(d,d+h),x=new Uint32Array(g),m=new Int32Array(g),p=new Array(f),T=0;T<f;T++){var S=T*2,A=x[S],B=m[S+1];p[T]=A+e*B}return p}function o(u,h){var d=u.byteLength,f=u.byteOffset,g=u.buffer.slice(f,f+d);return new h(g)}var a={64:Uint8Array,69:Uint16Array,70:Uint32Array,72:Int8Array,77:Int16Array,78:Int32Array,85:Float32Array,86:Float64Array},l={71:r,79:s};function c(u,h){if(h in a){var d=a[h];return o(u,d)}return h in l?l[h](u):u}i.exports&&(i.exports=c)}(Hr)),Hr.exports}var Wr,qa;function ku(){if(qa)return Wr;qa=1;var i=Ou(),e=Fu(),t=Bu(),n=null;typeof bson<"u"&&(n=bson().BSON);function r(s){var o=null;s.transportOptions.decoder&&(o=s.transportOptions.decoder);function a(u){u.op==="publish"?s.emit(u.topic,u.msg):u.op==="service_response"?s.emit(u.id,u):u.op==="call_service"?s.emit(u.service,u):u.op==="status"&&(u.id?s.emit("status:"+u.id,u):s.emit("status",u))}function l(u,h){u.op==="png"?i(u.data,h):h(u)}function c(u,h){if(!n)throw"Cannot process BSON encoded message without BSON header.";var d=new FileReader;d.onload=function(){var f=new Uint8Array(this.result),g=n.deserialize(f);h(g)},d.readAsArrayBuffer(u)}return{onopen:function(h){s.isConnected=!0,s.emit("connection",h)},onclose:function(h){s.isConnected=!1,s.emit("close",h)},onerror:function(h){s.emit("error",h)},onmessage:function(h){if(o)o(h.data,function(g){a(g)});else if(typeof Blob<"u"&&h.data instanceof Blob)c(h.data,function(g){l(g,a)});else if(h.data instanceof ArrayBuffer){var d=e.decode(h.data,t);a(d)}else{var f=JSON.parse(typeof h=="string"?h:h.data);l(f,a)}}}}return Wr=r,Wr}var Xr,Ya;function _c(){if(Ya)return Xr;Ya=1;var i=Nn;function e(t){i(this,t)}return Xr=e,Xr}var qr,ja;function Ai(){if(ja)return qr;ja=1;var i=Nn;function e(t){i(this,t)}return qr=e,qr}var Yr={exports:{}};/*!
 * EventEmitter2
 * https://github.com/hij1nx/EventEmitter2
 *
 * Copyright (c) 2013 hij1nx
 * Licensed under the MIT license.
 */var $a;function On(){return $a||($a=1,function(i,e){(function(t){var n=Object.hasOwnProperty,r=Array.isArray?Array.isArray:function(w){return Object.prototype.toString.call(w)==="[object Array]"},s=10,o=typeof process=="object"&&typeof process.nextTick=="function",a=typeof Symbol=="function",l=typeof Reflect=="object",c=typeof setImmediate=="function",u=c?setImmediate:setTimeout,h=a?l&&typeof Reflect.ownKeys=="function"?Reflect.ownKeys:function(v){var w=Object.getOwnPropertyNames(v);return w.push.apply(w,Object.getOwnPropertySymbols(v)),w}:Object.keys;function d(){this._events={},this._conf&&f.call(this,this._conf)}function f(v){v&&(this._conf=v,v.delimiter&&(this.delimiter=v.delimiter),v.maxListeners!==t&&(this._maxListeners=v.maxListeners),v.wildcard&&(this.wildcard=v.wildcard),v.newListener&&(this._newListener=v.newListener),v.removeListener&&(this._removeListener=v.removeListener),v.verboseMemoryLeak&&(this.verboseMemoryLeak=v.verboseMemoryLeak),v.ignoreErrors&&(this.ignoreErrors=v.ignoreErrors),this.wildcard&&(this.listenerTree={}))}function g(v,w){var U="(node) warning: possible EventEmitter memory leak detected. "+v+" listeners added. Use emitter.setMaxListeners() to increase limit.";if(this.verboseMemoryLeak&&(U+=" Event name: "+w+"."),typeof process<"u"&&process.emitWarning){var N=new Error(U);N.name="MaxListenersExceededWarning",N.emitter=this,N.count=v,process.emitWarning(N)}else console.error(U),console.trace&&console.trace()}var x=function(v,w,U){var N=arguments.length;switch(N){case 0:return[];case 1:return[v];case 2:return[v,w];case 3:return[v,w,U];default:for(var G=new Array(N);N--;)G[N]=arguments[N];return G}};function m(v,w){for(var U={},N,G=v.length,K=0,I=0;I<G;I++)N=v[I],U[N]=I<K?w[I]:t;return U}function p(v,w,U){this._emitter=v,this._target=w,this._listeners={},this._listenersCount=0;var N,G;if((U.on||U.off)&&(N=U.on,G=U.off),w.addEventListener?(N=w.addEventListener,G=w.removeEventListener):w.addListener?(N=w.addListener,G=w.removeListener):w.on&&(N=w.on,G=w.off),!N&&!G)throw Error("target does not implement any known event API");if(typeof N!="function")throw TypeError("on method must be a function");if(typeof G!="function")throw TypeError("off method must be a function");this._on=N,this._off=G;var K=v._observers;K?K.push(this):v._observers=[this]}Object.assign(p.prototype,{subscribe:function(v,w,U){var N=this,G=this._target,K=this._emitter,I=this._listeners,k=function(){var j=x.apply(null,arguments),q={data:j,name:w,original:v};if(U){var le=U.call(G,q);le!==!1&&K.emit.apply(K,[q.name].concat(j));return}K.emit.apply(K,[w].concat(j))};if(I[v])throw Error("Event '"+v+"' is already listening");this._listenersCount++,K._newListener&&K._removeListener&&!N._onNewListener?(this._onNewListener=function(j){j===w&&I[v]===null&&(I[v]=k,N._on.call(G,v,k))},K.on("newListener",this._onNewListener),this._onRemoveListener=function(j){j===w&&!K.hasListeners(j)&&I[v]&&(I[v]=null,N._off.call(G,v,k))},I[v]=null,K.on("removeListener",this._onRemoveListener)):(I[v]=k,N._on.call(G,v,k))},unsubscribe:function(v){var w=this,U=this._listeners,N=this._emitter,G,K,I=this._off,k=this._target,j;if(v&&typeof v!="string")throw TypeError("event must be a string");function q(){w._onNewListener&&(N.off("newListener",w._onNewListener),N.off("removeListener",w._onRemoveListener),w._onNewListener=null,w._onRemoveListener=null);var le=X.call(N,w);N._observers.splice(le,1)}if(v){if(G=U[v],!G)return;I.call(k,v,G),delete U[v],--this._listenersCount||q()}else{for(K=h(U),j=K.length;j-- >0;)v=K[j],I.call(k,v,U[v]);this._listeners={},this._listenersCount=0,q()}}});function T(v,w,U,N){var G=Object.assign({},w);if(!v)return G;if(typeof v!="object")throw TypeError("options must be an object");var K=Object.keys(v),I=K.length,k,j,q;function le(ge){throw Error('Invalid "'+k+'" option value'+(ge?". Reason: "+ge:""))}for(var ve=0;ve<I;ve++){if(k=K[ve],!n.call(w,k))throw Error('Unknown "'+k+'" option');j=v[k],j!==t&&(q=U[k],G[k]=q?q(j,le):j)}return G}function S(v,w){return(typeof v!="function"||!v.hasOwnProperty("prototype"))&&w("value must be a constructor"),v}function A(v){var w="value must be type of "+v.join("|"),U=v.length,N=v[0],G=v[1];return U===1?function(K,I){if(typeof K===N)return K;I(w)}:U===2?function(K,I){var k=typeof K;if(k===N||k===G)return K;I(w)}:function(K,I){for(var k=typeof K,j=U;j-- >0;)if(k===v[j])return K;I(w)}}var B=A(["function"]),D=A(["object","function"]);function P(v,w,U){var N,G,K=0,I,k=new v(function(j,q,le){U=T(U,{timeout:0,overload:!1},{timeout:function(xe,Te){return xe*=1,(typeof xe!="number"||xe<0||!Number.isFinite(xe))&&Te("timeout must be a positive number"),xe}}),N=!U.overload&&typeof v.prototype.cancel=="function"&&typeof le=="function";function ve(){G&&(G=null),K&&(clearTimeout(K),K=0)}var ge=function(xe){ve(),j(xe)},L=function(xe){ve(),q(xe)};N?w(ge,L,le):(G=[function(xe){L(xe||Error("canceled"))}],w(ge,L,function(xe){if(I)throw Error("Unable to subscribe on cancel event asynchronously");if(typeof xe!="function")throw TypeError("onCancel callback must be a function");G.push(xe)}),I=!0),U.timeout>0&&(K=setTimeout(function(){var xe=Error("timeout");xe.code="ETIMEDOUT",K=0,k.cancel(xe),q(xe)},U.timeout))});return N||(k.cancel=function(j){if(G){for(var q=G.length,le=1;le<q;le++)G[le](j);G[0](j),G=null}}),k}function X(v){var w=this._observers;if(!w)return-1;for(var U=w.length,N=0;N<U;N++)if(w[N]._target===v)return N;return-1}function E(v,w,U,N,G){if(!U)return null;if(N===0){var K=typeof w;if(K==="string"){var I,k,j=0,q=0,le=this.delimiter,ve=le.length;if((k=w.indexOf(le))!==-1){I=new Array(5);do I[j++]=w.slice(q,k),q=k+ve;while((k=w.indexOf(le,q))!==-1);I[j++]=w.slice(q),w=I,G=j}else w=[w],G=1}else K==="object"?G=w.length:(w=[w],G=1)}var ge=null,L,xe,Te,Xe,Me,Be=w[N],Ie=w[N+1],Ae,Ve;if(N===G)U._listeners&&(typeof U._listeners=="function"?(v&&v.push(U._listeners),ge=[U]):(v&&v.push.apply(v,U._listeners),ge=[U]));else if(Be==="*"){for(Ae=h(U),k=Ae.length;k-- >0;)L=Ae[k],L!=="_listeners"&&(Ve=E(v,w,U[L],N+1,G),Ve&&(ge?ge.push.apply(ge,Ve):ge=Ve));return ge}else if(Be==="**"){for(Me=N+1===G||N+2===G&&Ie==="*",Me&&U._listeners&&(ge=E(v,w,U,G,G)),Ae=h(U),k=Ae.length;k-- >0;)L=Ae[k],L!=="_listeners"&&(L==="*"||L==="**"?(U[L]._listeners&&!Me&&(Ve=E(v,w,U[L],G,G),Ve&&(ge?ge.push.apply(ge,Ve):ge=Ve)),Ve=E(v,w,U[L],N,G)):L===Ie?Ve=E(v,w,U[L],N+2,G):Ve=E(v,w,U[L],N,G),Ve&&(ge?ge.push.apply(ge,Ve):ge=Ve));return ge}else U[Be]&&(ge=E(v,w,U[Be],N+1,G));if(xe=U["*"],xe&&E(v,w,xe,N+1,G),Te=U["**"],Te)if(N<G)for(Te._listeners&&E(v,w,Te,G,G),Ae=h(Te),k=Ae.length;k-- >0;)L=Ae[k],L!=="_listeners"&&(L===Ie?E(v,w,Te[L],N+2,G):L===Be?E(v,w,Te[L],N+1,G):(Xe={},Xe[L]=Te[L],E(v,w,{"**":Xe},N+1,G)));else Te._listeners?E(v,w,Te,G,G):Te["*"]&&Te["*"]._listeners&&E(v,w,Te["*"],G,G);return ge}function M(v,w,U){var N=0,G=0,K,I=this.delimiter,k=I.length,j;if(typeof v=="string")if((K=v.indexOf(I))!==-1){j=new Array(5);do j[N++]=v.slice(G,K),G=K+k;while((K=v.indexOf(I,G))!==-1);j[N++]=v.slice(G)}else j=[v],N=1;else j=v,N=v.length;if(N>1){for(K=0;K+1<N;K++)if(j[K]==="**"&&j[K+1]==="**")return}var q=this.listenerTree,le;for(K=0;K<N;K++)if(le=j[K],q=q[le]||(q[le]={}),K===N-1)return q._listeners?(typeof q._listeners=="function"&&(q._listeners=[q._listeners]),U?q._listeners.unshift(w):q._listeners.push(w),!q._listeners.warned&&this._maxListeners>0&&q._listeners.length>this._maxListeners&&(q._listeners.warned=!0,g.call(this,q._listeners.length,le))):q._listeners=w,!0;return!0}function C(v,w,U,N){for(var G=h(v),K=G.length,I,k,j,q=v._listeners,le;K-- >0;)k=G[K],I=v[k],k==="_listeners"?j=U:j=U?U.concat(k):[k],le=N||typeof k=="symbol",q&&w.push(le?j:j.join(this.delimiter)),typeof I=="object"&&C.call(this,I,w,j,le);return w}function H(v){for(var w=h(v),U=w.length,N,G,K;U-- >0;)G=w[U],N=v[G],N&&(K=!0,G!=="_listeners"&&!H(N)&&delete v[G]);return K}function O(v,w,U){this.emitter=v,this.event=w,this.listener=U}O.prototype.off=function(){return this.emitter.off(this.event,this.listener),this};function $(v,w,U){if(U===!0)G=!0;else if(U===!1)N=!0;else{if(!U||typeof U!="object")throw TypeError("options should be an object or true");var N=U.async,G=U.promisify,K=U.nextTick,I=U.objectify}if(N||K||G){var k=w,j=w._origin||w;if(K&&!o)throw Error("process.nextTick is not supported");G===t&&(G=w.constructor.name==="AsyncFunction"),w=function(){var q=arguments,le=this,ve=this.event;return G?K?Promise.resolve():new Promise(function(ge){u(ge)}).then(function(){return le.event=ve,k.apply(le,q)}):(K?process.nextTick:u)(function(){le.event=ve,k.apply(le,q)})},w._async=!0,w._origin=j}return[w,I?new O(this,v,w):this]}function V(v){this._events={},this._newListener=!1,this._removeListener=!1,this.verboseMemoryLeak=!1,f.call(this,v)}V.EventEmitter2=V,V.prototype.listenTo=function(v,w,U){if(typeof v!="object")throw TypeError("target musts be an object");var N=this;U=T(U,{on:t,off:t,reducers:t},{on:B,off:B,reducers:D});function G(K){if(typeof K!="object")throw TypeError("events must be an object");var I=U.reducers,k=X.call(N,v),j;k===-1?j=new p(N,v,U):j=N._observers[k];for(var q=h(K),le=q.length,ve,ge=typeof I=="function",L=0;L<le;L++)ve=q[L],j.subscribe(ve,K[ve]||ve,ge?I:I&&I[ve])}return r(w)?G(m(w)):G(typeof w=="string"?m(w.split(/\s+/)):w),this},V.prototype.stopListeningTo=function(v,w){var U=this._observers;if(!U)return!1;var N=U.length,G,K=!1;if(v&&typeof v!="object")throw TypeError("target should be an object");for(;N-- >0;)G=U[N],(!v||G._target===v)&&(G.unsubscribe(w),K=!0);return K},V.prototype.delimiter=".",V.prototype.setMaxListeners=function(v){v!==t&&(this._maxListeners=v,this._conf||(this._conf={}),this._conf.maxListeners=v)},V.prototype.getMaxListeners=function(){return this._maxListeners},V.prototype.event="",V.prototype.once=function(v,w,U){return this._once(v,w,!1,U)},V.prototype.prependOnceListener=function(v,w,U){return this._once(v,w,!0,U)},V.prototype._once=function(v,w,U,N){return this._many(v,1,w,U,N)},V.prototype.many=function(v,w,U,N){return this._many(v,w,U,!1,N)},V.prototype.prependMany=function(v,w,U,N){return this._many(v,w,U,!0,N)},V.prototype._many=function(v,w,U,N,G){var K=this;if(typeof U!="function")throw new Error("many only accepts instances of Function");function I(){return--w===0&&K.off(v,I),U.apply(this,arguments)}return I._origin=U,this._on(v,I,N,G)},V.prototype.emit=function(){if(!this._events&&!this._all)return!1;this._events||d.call(this);var v=arguments[0],w,U=this.wildcard,N,G,K,I,k;if(v==="newListener"&&!this._newListener&&!this._events.newListener)return!1;if(U&&(w=v,v!=="newListener"&&v!=="removeListener"&&typeof v=="object")){if(G=v.length,a){for(K=0;K<G;K++)if(typeof v[K]=="symbol"){k=!0;break}}k||(v=v.join(this.delimiter))}var j=arguments.length,q;if(this._all&&this._all.length)for(q=this._all.slice(),K=0,G=q.length;K<G;K++)switch(this.event=v,j){case 1:q[K].call(this,v);break;case 2:q[K].call(this,v,arguments[1]);break;case 3:q[K].call(this,v,arguments[1],arguments[2]);break;default:q[K].apply(this,arguments)}if(U)q=[],E.call(this,q,w,this.listenerTree,0,G);else if(q=this._events[v],typeof q=="function"){switch(this.event=v,j){case 1:q.call(this);break;case 2:q.call(this,arguments[1]);break;case 3:q.call(this,arguments[1],arguments[2]);break;default:for(N=new Array(j-1),I=1;I<j;I++)N[I-1]=arguments[I];q.apply(this,N)}return!0}else q&&(q=q.slice());if(q&&q.length){if(j>3)for(N=new Array(j-1),I=1;I<j;I++)N[I-1]=arguments[I];for(K=0,G=q.length;K<G;K++)switch(this.event=v,j){case 1:q[K].call(this);break;case 2:q[K].call(this,arguments[1]);break;case 3:q[K].call(this,arguments[1],arguments[2]);break;default:q[K].apply(this,N)}return!0}else if(!this.ignoreErrors&&!this._all&&v==="error")throw arguments[1]instanceof Error?arguments[1]:new Error("Uncaught, unspecified 'error' event.");return!!this._all},V.prototype.emitAsync=function(){if(!this._events&&!this._all)return!1;this._events||d.call(this);var v=arguments[0],w=this.wildcard,U,N,G,K,I,k;if(v==="newListener"&&!this._newListener&&!this._events.newListener)return Promise.resolve([!1]);if(w&&(U=v,v!=="newListener"&&v!=="removeListener"&&typeof v=="object")){if(K=v.length,a){for(I=0;I<K;I++)if(typeof v[I]=="symbol"){N=!0;break}}N||(v=v.join(this.delimiter))}var j=[],q=arguments.length,le;if(this._all)for(I=0,K=this._all.length;I<K;I++)switch(this.event=v,q){case 1:j.push(this._all[I].call(this,v));break;case 2:j.push(this._all[I].call(this,v,arguments[1]));break;case 3:j.push(this._all[I].call(this,v,arguments[1],arguments[2]));break;default:j.push(this._all[I].apply(this,arguments))}if(w?(le=[],E.call(this,le,U,this.listenerTree,0)):le=this._events[v],typeof le=="function")switch(this.event=v,q){case 1:j.push(le.call(this));break;case 2:j.push(le.call(this,arguments[1]));break;case 3:j.push(le.call(this,arguments[1],arguments[2]));break;default:for(G=new Array(q-1),k=1;k<q;k++)G[k-1]=arguments[k];j.push(le.apply(this,G))}else if(le&&le.length){if(le=le.slice(),q>3)for(G=new Array(q-1),k=1;k<q;k++)G[k-1]=arguments[k];for(I=0,K=le.length;I<K;I++)switch(this.event=v,q){case 1:j.push(le[I].call(this));break;case 2:j.push(le[I].call(this,arguments[1]));break;case 3:j.push(le[I].call(this,arguments[1],arguments[2]));break;default:j.push(le[I].apply(this,G))}}else if(!this.ignoreErrors&&!this._all&&v==="error")return arguments[1]instanceof Error?Promise.reject(arguments[1]):Promise.reject("Uncaught, unspecified 'error' event.");return Promise.all(j)},V.prototype.on=function(v,w,U){return this._on(v,w,!1,U)},V.prototype.prependListener=function(v,w,U){return this._on(v,w,!0,U)},V.prototype.onAny=function(v){return this._onAny(v,!1)},V.prototype.prependAny=function(v){return this._onAny(v,!0)},V.prototype.addListener=V.prototype.on,V.prototype._onAny=function(v,w){if(typeof v!="function")throw new Error("onAny only accepts instances of Function");return this._all||(this._all=[]),w?this._all.unshift(v):this._all.push(v),this},V.prototype._on=function(v,w,U,N){if(typeof v=="function")return this._onAny(v,w),this;if(typeof w!="function")throw new Error("on only accepts instances of Function");this._events||d.call(this);var G=this,K;return N!==t&&(K=$.call(this,v,w,N),w=K[0],G=K[1]),this._newListener&&this.emit("newListener",v,w),this.wildcard?(M.call(this,v,w,U),G):(this._events[v]?(typeof this._events[v]=="function"&&(this._events[v]=[this._events[v]]),U?this._events[v].unshift(w):this._events[v].push(w),!this._events[v].warned&&this._maxListeners>0&&this._events[v].length>this._maxListeners&&(this._events[v].warned=!0,g.call(this,this._events[v].length,v))):this._events[v]=w,G)},V.prototype.off=function(v,w){if(typeof w!="function")throw new Error("removeListener only takes instances of Function");var U,N=[];if(this.wildcard){var G=typeof v=="string"?v.split(this.delimiter):v.slice();if(N=E.call(this,null,G,this.listenerTree,0),!N)return this}else{if(!this._events[v])return this;U=this._events[v],N.push({_listeners:U})}for(var K=0;K<N.length;K++){var I=N[K];if(U=I._listeners,r(U)){for(var k=-1,j=0,q=U.length;j<q;j++)if(U[j]===w||U[j].listener&&U[j].listener===w||U[j]._origin&&U[j]._origin===w){k=j;break}if(k<0)continue;return this.wildcard?I._listeners.splice(k,1):this._events[v].splice(k,1),U.length===0&&(this.wildcard?delete I._listeners:delete this._events[v]),this._removeListener&&this.emit("removeListener",v,w),this}else(U===w||U.listener&&U.listener===w||U._origin&&U._origin===w)&&(this.wildcard?delete I._listeners:delete this._events[v],this._removeListener&&this.emit("removeListener",v,w))}return this.listenerTree&&H(this.listenerTree),this},V.prototype.offAny=function(v){var w=0,U=0,N;if(v&&this._all&&this._all.length>0){for(N=this._all,w=0,U=N.length;w<U;w++)if(v===N[w])return N.splice(w,1),this._removeListener&&this.emit("removeListenerAny",v),this}else{if(N=this._all,this._removeListener)for(w=0,U=N.length;w<U;w++)this.emit("removeListenerAny",N[w]);this._all=[]}return this},V.prototype.removeListener=V.prototype.off,V.prototype.removeAllListeners=function(v){if(v===t)return!this._events||d.call(this),this;if(this.wildcard){var w=E.call(this,null,v,this.listenerTree,0),U,N;if(!w)return this;for(N=0;N<w.length;N++)U=w[N],U._listeners=null;this.listenerTree&&H(this.listenerTree)}else this._events&&(this._events[v]=null);return this},V.prototype.listeners=function(v){var w=this._events,U,N,G,K,I;if(v===t){if(this.wildcard)throw Error("event name required for wildcard emitter");if(!w)return[];for(U=h(w),K=U.length,G=[];K-- >0;)N=w[U[K]],typeof N=="function"?G.push(N):G.push.apply(G,N);return G}else{if(this.wildcard){if(I=this.listenerTree,!I)return[];var k=[],j=typeof v=="string"?v.split(this.delimiter):v.slice();return E.call(this,k,j,I,0),k}return w?(N=w[v],N?typeof N=="function"?[N]:N:[]):[]}},V.prototype.eventNames=function(v){var w=this._events;return this.wildcard?C.call(this,this.listenerTree,[],null,v):w?h(w):[]},V.prototype.listenerCount=function(v){return this.listeners(v).length},V.prototype.hasListeners=function(v){if(this.wildcard){var w=[],U=typeof v=="string"?v.split(this.delimiter):v.slice();return E.call(this,w,U,this.listenerTree,0),w.length>0}var N=this._events,G=this._all;return!!(G&&G.length||N&&(v===t?h(N).length:N[v]))},V.prototype.listenersAny=function(){return this._all?this._all:[]},V.prototype.waitFor=function(v,w){var U=this,N=typeof w;return N==="number"?w={timeout:w}:N==="function"&&(w={filter:w}),w=T(w,{timeout:0,filter:t,handleError:!1,Promise,overload:!1},{filter:B,Promise:S}),P(w.Promise,function(G,K,I){function k(){var j=w.filter;if(!(j&&!j.apply(U,arguments)))if(U.off(v,k),w.handleError){var q=arguments[0];q?K(q):G(x.apply(null,arguments).slice(1))}else G(x.apply(null,arguments))}I(function(){U.off(v,k)}),U._on(v,k,!1)},{timeout:w.timeout,overload:w.overload})};function Y(v,w,U){U=T(U,{Promise,timeout:0,overload:!1},{Promise:S});var N=U.Promise;return P(N,function(G,K,I){var k;if(typeof v.addEventListener=="function"){k=function(){G(x.apply(null,arguments))},I(function(){v.removeEventListener(w,k)}),v.addEventListener(w,k,{once:!0});return}var j=function(){q&&v.removeListener("error",q),G(x.apply(null,arguments))},q;w!=="error"&&(q=function(le){v.removeListener(w,j),K(le)},v.once("error",q)),I(function(){q&&v.removeListener("error",q),v.removeListener(w,j)}),v.once(w,j)},{timeout:U.timeout,overload:U.overload})}var te=V.prototype;Object.defineProperties(V,{defaultMaxListeners:{get:function(){return te._maxListeners},set:function(v){if(typeof v!="number"||v<0||Number.isNaN(v))throw TypeError("n must be a non-negative number");te._maxListeners=v},enumerable:!0},once:{value:Y,writable:!0,configurable:!0}}),Object.defineProperties(te,{_maxListeners:{value:s,writable:!0,configurable:!0},_observers:{value:null,writable:!0,configurable:!0}}),i.exports=V})()}(Yr)),Yr.exports}var jr,Ka;function br(){if(Ka)return jr;Ka=1;var i=_c();Ai();var e=On().EventEmitter2;function t(n){n=n||{},this.ros=n.ros,this.name=n.name,this.serviceType=n.serviceType,this.isAdvertised=!1,this._serviceCallback=null}return t.prototype.__proto__=e.prototype,t.prototype.callService=function(n,r,s){if(!this.isAdvertised){var o="call_service:"+this.name+":"+ ++this.ros.idCounter;(r||s)&&this.ros.once(o,function(l){l.result!==void 0&&l.result===!1?typeof s=="function"&&s(l.values):typeof r=="function"&&r(new i(l.values))});var a={op:"call_service",id:o,service:this.name,type:this.serviceType,args:n};this.ros.callOnConnection(a)}},t.prototype.advertise=function(n){this.isAdvertised||typeof n!="function"||(this._serviceCallback=n,this.ros.on(this.name,this._serviceResponse.bind(this)),this.ros.callOnConnection({op:"advertise_service",type:this.serviceType,service:this.name}),this.isAdvertised=!0)},t.prototype.unadvertise=function(){this.isAdvertised&&(this.ros.callOnConnection({op:"unadvertise_service",service:this.name}),this.isAdvertised=!1)},t.prototype._serviceResponse=function(n){var r={},s=this._serviceCallback(n.args,r),o={op:"service_response",service:this.name,values:new i(r),result:s};n.id&&(o.id=n.id),this.ros.callOnConnection(o)},jr=t,jr}var $r,Za;function Sa(){if(Za)return $r;Za=1;var i=gc(),e=Iu(),t=ku(),n=br(),r=Ai(),s=Nn,o=On().EventEmitter2;function a(l){l=l||{};var c=this;this.socket=null,this.idCounter=0,this.isConnected=!1,this.transportLibrary=l.transportLibrary||"websocket",this.transportOptions=l.transportOptions||{},this._sendFunc=function(u){c.sendEncodedMessage(u)},typeof l.groovyCompatibility>"u"?this.groovyCompatibility=!0:this.groovyCompatibility=l.groovyCompatibility,this.setMaxListeners(0),l.url&&this.connect(l.url)}return a.prototype.__proto__=o.prototype,a.prototype.connect=function(l){if(this.transportLibrary==="socket.io")this.socket=s(io(l,{"force new connection":!0}),t(this)),this.socket.on("connect",this.socket.onopen),this.socket.on("data",this.socket.onmessage),this.socket.on("close",this.socket.onclose),this.socket.on("error",this.socket.onerror);else if(this.transportLibrary.constructor.name==="RTCPeerConnection")this.socket=s(this.transportLibrary.createDataChannel(l,this.transportOptions),t(this));else if(this.transportLibrary==="websocket"){if(!this.socket||this.socket.readyState===i.CLOSED){var c=new i(l);c.binaryType="arraybuffer",this.socket=s(c,t(this))}}else if(this.transportLibrary==="workersocket")this.socket=s(new e(l),t(this));else throw"Unknown transportLibrary: "+this.transportLibrary.toString()},a.prototype.close=function(){this.socket&&this.socket.close()},a.prototype.authenticate=function(l,c,u,h,d,f,g){var x={op:"auth",mac:l,client:c,dest:u,rand:h,t:d,level:f,end:g};this.callOnConnection(x)},a.prototype.sendEncodedMessage=function(l){var c=null,u=this;this.transportLibrary==="socket.io"?c=function(h){u.socket.emit("operation",h)}:c=function(h){u.socket.send(h)},this.isConnected?c(l):u.once("connection",function(){c(l)})},a.prototype.callOnConnection=function(l){this.transportOptions.encoder?this.transportOptions.encoder(l,this._sendFunc):this._sendFunc(JSON.stringify(l))},a.prototype.setStatusLevel=function(l,c){var u={op:"set_level",level:l,id:c};this.callOnConnection(u)},a.prototype.getActionServers=function(l,c){var u=new n({ros:this,name:"/rosapi/action_servers",serviceType:"rosapi/GetActionServers"}),h=new r({});typeof c=="function"?u.callService(h,function(d){l(d.action_servers)},function(d){c(d)}):u.callService(h,function(d){l(d.action_servers)})},a.prototype.getTopics=function(l,c){var u=new n({ros:this,name:"/rosapi/topics",serviceType:"rosapi/Topics"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d)},function(d){c(d)}):u.callService(h,function(d){l(d)})},a.prototype.getTopicsForType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/topics_for_type",serviceType:"rosapi/TopicsForType"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f.topics)},function(f){u(f)}):h.callService(d,function(f){c(f.topics)})},a.prototype.getServices=function(l,c){var u=new n({ros:this,name:"/rosapi/services",serviceType:"rosapi/Services"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d.services)},function(d){c(d)}):u.callService(h,function(d){l(d.services)})},a.prototype.getServicesForType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/services_for_type",serviceType:"rosapi/ServicesForType"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f.services)},function(f){u(f)}):h.callService(d,function(f){c(f.services)})},a.prototype.getServiceRequestDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/service_request_details",serviceType:"rosapi/ServiceRequestDetails"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f)},function(f){u(f)}):h.callService(d,function(f){c(f)})},a.prototype.getServiceResponseDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/service_response_details",serviceType:"rosapi/ServiceResponseDetails"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f)},function(f){u(f)}):h.callService(d,function(f){c(f)})},a.prototype.getNodes=function(l,c){var u=new n({ros:this,name:"/rosapi/nodes",serviceType:"rosapi/Nodes"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d.nodes)},function(d){c(d)}):u.callService(h,function(d){l(d.nodes)})},a.prototype.getNodeDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/node_details",serviceType:"rosapi/NodeDetails"}),d=new r({node:l});typeof u=="function"?h.callService(d,function(f){c(f.subscribing,f.publishing,f.services)},function(f){u(f)}):h.callService(d,function(f){c(f)})},a.prototype.getParams=function(l,c){var u=new n({ros:this,name:"/rosapi/get_param_names",serviceType:"rosapi/GetParamNames"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d.names)},function(d){c(d)}):u.callService(h,function(d){l(d.names)})},a.prototype.getTopicType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/topic_type",serviceType:"rosapi/TopicType"}),d=new r({topic:l});typeof u=="function"?h.callService(d,function(f){c(f.type)},function(f){u(f)}):h.callService(d,function(f){c(f.type)})},a.prototype.getServiceType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/service_type",serviceType:"rosapi/ServiceType"}),d=new r({service:l});typeof u=="function"?h.callService(d,function(f){c(f.type)},function(f){u(f)}):h.callService(d,function(f){c(f.type)})},a.prototype.getMessageDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/message_details",serviceType:"rosapi/MessageDetails"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f.typedefs)},function(f){u(f)}):h.callService(d,function(f){c(f.typedefs)})},a.prototype.decodeTypeDefs=function(l){var c=this,u=function(h,d){for(var f={},g=0;g<h.fieldnames.length;g++){var x=h.fieldarraylen[g],m=h.fieldnames[g],p=h.fieldtypes[g];if(p.indexOf("/")===-1)x===-1?f[m]=p:f[m]=[p];else{for(var T=!1,S=0;S<d.length;S++)if(d[S].type.toString()===p.toString()){T=d[S];break}if(T){var A=u(T,d);x===-1?f[m]=A:f[m]=[A]}else c.emit("error","Cannot find "+p+" in decodeTypeDefs")}}return f};return u(l[0],l)},a.prototype.getTopicsAndRawTypes=function(l,c){var u=new n({ros:this,name:"/rosapi/topics_and_raw_types",serviceType:"rosapi/TopicsAndRawTypes"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d)},function(d){c(d)}):u.callService(h,function(d){l(d)})},$r=a,$r}var Kr,Ja;function gi(){if(Ja)return Kr;Ja=1;var i=Nn;function e(t){i(this,t)}return Kr=e,Kr}var Zr,Qa;function Ci(){if(Qa)return Zr;Qa=1;var i=On().EventEmitter2,e=gi();function t(n){n=n||{},this.ros=n.ros,this.name=n.name,this.messageType=n.messageType,this.isAdvertised=!1,this.compression=n.compression||"none",this.throttle_rate=n.throttle_rate||0,this.latch=n.latch||!1,this.queue_size=n.queue_size||100,this.queue_length=n.queue_length||0,this.reconnect_on_close=n.reconnect_on_close!==void 0?n.reconnect_on_close:!0,this.compression&&this.compression!=="png"&&this.compression!=="cbor"&&this.compression!=="cbor-raw"&&this.compression!=="none"&&(this.emit("warning",this.compression+" compression is not supported. No compression will be used."),this.compression="none"),this.throttle_rate<0&&(this.emit("warning",this.throttle_rate+" is not allowed. Set to 0"),this.throttle_rate=0);var r=this;this.reconnect_on_close?this.callForSubscribeAndAdvertise=function(s){r.ros.callOnConnection(s),r.waitForReconnect=!1,r.reconnectFunc=function(){r.waitForReconnect||(r.waitForReconnect=!0,r.ros.callOnConnection(s),r.ros.once("connection",function(){r.waitForReconnect=!1}))},r.ros.on("close",r.reconnectFunc)}:this.callForSubscribeAndAdvertise=this.ros.callOnConnection,this._messageCallback=function(s){r.emit("message",new e(s))}}return t.prototype.__proto__=i.prototype,t.prototype.subscribe=function(n){typeof n=="function"&&this.on("message",n),!this.subscribeId&&(this.ros.on(this.name,this._messageCallback),this.subscribeId="subscribe:"+this.name+":"+ ++this.ros.idCounter,this.callForSubscribeAndAdvertise({op:"subscribe",id:this.subscribeId,type:this.messageType,topic:this.name,compression:this.compression,throttle_rate:this.throttle_rate,queue_length:this.queue_length}))},t.prototype.unsubscribe=function(n){n&&(this.off("message",n),this.listeners("message").length)||this.subscribeId&&(this.ros.off(this.name,this._messageCallback),this.reconnect_on_close&&this.ros.off("close",this.reconnectFunc),this.emit("unsubscribe"),this.ros.callOnConnection({op:"unsubscribe",id:this.subscribeId,topic:this.name}),this.subscribeId=null)},t.prototype.advertise=function(){if(!this.isAdvertised&&(this.advertiseId="advertise:"+this.name+":"+ ++this.ros.idCounter,this.callForSubscribeAndAdvertise({op:"advertise",id:this.advertiseId,type:this.messageType,topic:this.name,latch:this.latch,queue_size:this.queue_size}),this.isAdvertised=!0,!this.reconnect_on_close)){var n=this;this.ros.on("close",function(){n.isAdvertised=!1})}},t.prototype.unadvertise=function(){this.isAdvertised&&(this.reconnect_on_close&&this.ros.off("close",this.reconnectFunc),this.emit("unadvertise"),this.ros.callOnConnection({op:"unadvertise",id:this.advertiseId,topic:this.name}),this.isAdvertised=!1)},t.prototype.publish=function(n){this.isAdvertised||this.advertise(),this.ros.idCounter++;var r={op:"publish",id:"publish:"+this.name+":"+this.ros.idCounter,topic:this.name,msg:n,latch:this.latch};this.ros.callOnConnection(r)},Zr=t,Zr}var Jr,eo;function zu(){if(eo)return Jr;eo=1;var i=br(),e=Ai();function t(n){n=n||{},this.ros=n.ros,this.name=n.name}return t.prototype.get=function(n,r){var s=new i({ros:this.ros,name:"/rosapi/get_param",serviceType:"rosapi/GetParam"}),o=new e({name:this.name});s.callService(o,function(a){var l=JSON.parse(a.value);n(l)},r)},t.prototype.set=function(n,r,s){var o=new i({ros:this.ros,name:"/rosapi/set_param",serviceType:"rosapi/SetParam"}),a=new e({name:this.name,value:JSON.stringify(n)});o.callService(a,r,s)},t.prototype.delete=function(n,r){var s=new i({ros:this.ros,name:"/rosapi/delete_param",serviceType:"rosapi/DeleteParam"}),o=new e({name:this.name});s.callService(o,n,r)},Jr=t,Jr}var to;function Vu(){if(to)return Dr.exports;to=1;var i=ya(),e=Dr.exports={Ros:Sa(),Topic:Ci(),Message:gi(),Param:zu(),Service:br(),ServiceRequest:Ai(),ServiceResponse:_c()};return i(e.Ros,["Param","Service","Topic"],e),Dr.exports}var Qr={exports:{}},es,no;function vc(){if(no)return es;no=1;var i=Ci(),e=gi(),t=On().EventEmitter2;function n(r){var s=this;r=r||{},this.ros=r.ros,this.serverName=r.serverName,this.actionName=r.actionName,this.timeout=r.timeout,this.omitFeedback=r.omitFeedback,this.omitStatus=r.omitStatus,this.omitResult=r.omitResult,this.goals={};var o=!1;this.feedbackListener=new i({ros:this.ros,name:this.serverName+"/feedback",messageType:this.actionName+"Feedback"}),this.statusListener=new i({ros:this.ros,name:this.serverName+"/status",messageType:"actionlib_msgs/GoalStatusArray"}),this.resultListener=new i({ros:this.ros,name:this.serverName+"/result",messageType:this.actionName+"Result"}),this.goalTopic=new i({ros:this.ros,name:this.serverName+"/goal",messageType:this.actionName+"Goal"}),this.cancelTopic=new i({ros:this.ros,name:this.serverName+"/cancel",messageType:"actionlib_msgs/GoalID"}),this.goalTopic.advertise(),this.cancelTopic.advertise(),this.omitStatus||this.statusListener.subscribe(function(a){o=!0,a.status_list.forEach(function(l){var c=s.goals[l.goal_id.id];c&&c.emit("status",l)})}),this.omitFeedback||this.feedbackListener.subscribe(function(a){var l=s.goals[a.status.goal_id.id];l&&(l.emit("status",a.status),l.emit("feedback",a.feedback))}),this.omitResult||this.resultListener.subscribe(function(a){var l=s.goals[a.status.goal_id.id];l&&(l.emit("status",a.status),l.emit("result",a.result))}),this.timeout&&setTimeout(function(){o||s.emit("timeout")},this.timeout)}return n.prototype.__proto__=t.prototype,n.prototype.cancel=function(){var r=new e;this.cancelTopic.publish(r)},n.prototype.dispose=function(){this.goalTopic.unadvertise(),this.cancelTopic.unadvertise(),this.omitStatus||this.statusListener.unsubscribe(),this.omitFeedback||this.feedbackListener.unsubscribe(),this.omitResult||this.resultListener.unsubscribe()},es=n,es}var ts,ro;function Gu(){if(ro)return ts;ro=1;var i=Ci();gi();var e=On().EventEmitter2;function t(n){var r=this;n=n||{},this.ros=n.ros,this.serverName=n.serverName,this.actionName=n.actionName;var s=new i({ros:this.ros,name:this.serverName+"/goal",messageType:this.actionName+"Goal"}),o=new i({ros:this.ros,name:this.serverName+"/feedback",messageType:this.actionName+"Feedback"}),a=new i({ros:this.ros,name:this.serverName+"/status",messageType:"actionlib_msgs/GoalStatusArray"}),l=new i({ros:this.ros,name:this.serverName+"/result",messageType:this.actionName+"Result"});s.subscribe(function(c){r.emit("goal",c)}),a.subscribe(function(c){c.status_list.forEach(function(u){r.emit("status",u)})}),o.subscribe(function(c){r.emit("status",c.status),r.emit("feedback",c.feedback)}),l.subscribe(function(c){r.emit("status",c.status),r.emit("result",c.result)})}return t.prototype.__proto__=e.prototype,ts=t,ts}var ns,so;function xc(){if(so)return ns;so=1;var i=gi(),e=On().EventEmitter2;function t(n){var r=this;this.actionClient=n.actionClient,this.goalMessage=n.goalMessage,this.isFinished=!1;var s=new Date;this.goalID="goal_"+Math.random()+"_"+s.getTime(),this.goalMessage=new i({goal_id:{stamp:{secs:0,nsecs:0},id:this.goalID},goal:this.goalMessage}),this.on("status",function(o){r.status=o}),this.on("result",function(o){r.isFinished=!0,r.result=o}),this.on("feedback",function(o){r.feedback=o}),this.actionClient.goals[this.goalID]=this}return t.prototype.__proto__=e.prototype,t.prototype.send=function(n){var r=this;r.actionClient.goalTopic.publish(r.goalMessage),n&&setTimeout(function(){r.isFinished||r.emit("timeout")},n)},t.prototype.cancel=function(){var n=new i({id:this.goalID});this.actionClient.cancelTopic.publish(n)},ns=t,ns}var is,ao;function Hu(){if(ao)return is;ao=1;var i=Ci(),e=gi(),t=On().EventEmitter2;function n(r){var s=this;r=r||{},this.ros=r.ros,this.serverName=r.serverName,this.actionName=r.actionName,this.feedbackPublisher=new i({ros:this.ros,name:this.serverName+"/feedback",messageType:this.actionName+"Feedback"}),this.feedbackPublisher.advertise();var o=new i({ros:this.ros,name:this.serverName+"/status",messageType:"actionlib_msgs/GoalStatusArray"});o.advertise(),this.resultPublisher=new i({ros:this.ros,name:this.serverName+"/result",messageType:this.actionName+"Result"}),this.resultPublisher.advertise();var a=new i({ros:this.ros,name:this.serverName+"/goal",messageType:this.actionName+"Goal"}),l=new i({ros:this.ros,name:this.serverName+"/cancel",messageType:"actionlib_msgs/GoalID"});this.statusMessage=new e({header:{stamp:{secs:0,nsecs:100},frame_id:""},status_list:[]}),this.currentGoal=null,this.nextGoal=null,a.subscribe(function(u){s.currentGoal?(s.nextGoal=u,s.emit("cancel")):(s.statusMessage.status_list=[{goal_id:u.goal_id,status:1}],s.currentGoal=u,s.emit("goal",u.goal))});var c=function(u,h){return u.secs>h.secs?!1:u.secs<h.secs?!0:u.nsecs<h.nsecs};l.subscribe(function(u){u.stamp.secs===0&&u.stamp.secs===0&&u.id===""?(s.nextGoal=null,s.currentGoal&&s.emit("cancel")):(s.currentGoal&&u.id===s.currentGoal.goal_id.id?s.emit("cancel"):s.nextGoal&&u.id===s.nextGoal.goal_id.id&&(s.nextGoal=null),s.nextGoal&&c(s.nextGoal.goal_id.stamp,u.stamp)&&(s.nextGoal=null),s.currentGoal&&c(s.currentGoal.goal_id.stamp,u.stamp)&&s.emit("cancel"))}),setInterval(function(){var u=new Date,h=Math.floor(u.getTime()/1e3),d=Math.round(1e9*(u.getTime()/1e3-h));s.statusMessage.header.stamp.secs=h,s.statusMessage.header.stamp.nsecs=d,o.publish(s.statusMessage)},500)}return n.prototype.__proto__=t.prototype,n.prototype.setSucceeded=function(r){var s=new e({status:{goal_id:this.currentGoal.goal_id,status:3},result:r});this.resultPublisher.publish(s),this.statusMessage.status_list=[],this.nextGoal?(this.currentGoal=this.nextGoal,this.nextGoal=null,this.emit("goal",this.currentGoal.goal)):this.currentGoal=null},n.prototype.setAborted=function(r){var s=new e({status:{goal_id:this.currentGoal.goal_id,status:4},result:r});this.resultPublisher.publish(s),this.statusMessage.status_list=[],this.nextGoal?(this.currentGoal=this.nextGoal,this.nextGoal=null,this.emit("goal",this.currentGoal.goal)):this.currentGoal=null},n.prototype.sendFeedback=function(r){var s=new e({status:{goal_id:this.currentGoal.goal_id,status:1},feedback:r});this.feedbackPublisher.publish(s)},n.prototype.setPreempted=function(){this.statusMessage.status_list=[];var r=new e({status:{goal_id:this.currentGoal.goal_id,status:2}});this.resultPublisher.publish(r),this.nextGoal?(this.currentGoal=this.nextGoal,this.nextGoal=null,this.emit("goal",this.currentGoal.goal)):this.currentGoal=null},is=n,is}var oo;function Wu(){if(oo)return Qr.exports;oo=1;var i=Sa(),e=ya(),t=Qr.exports={ActionClient:vc(),ActionListener:Gu(),Goal:xc(),SimpleActionServer:Hu()};return e(i,["ActionClient","SimpleActionServer"],t),Qr.exports}var rs,lo;function Fn(){if(lo)return rs;lo=1;function i(e){e=e||{},this.x=e.x||0,this.y=e.y||0,this.z=e.z||0}return i.prototype.add=function(e){this.x+=e.x,this.y+=e.y,this.z+=e.z},i.prototype.subtract=function(e){this.x-=e.x,this.y-=e.y,this.z-=e.z},i.prototype.multiplyQuaternion=function(e){var t=e.w*this.x+e.y*this.z-e.z*this.y,n=e.w*this.y+e.z*this.x-e.x*this.z,r=e.w*this.z+e.x*this.y-e.y*this.x,s=-e.x*this.x-e.y*this.y-e.z*this.z;this.x=t*e.w+s*-e.x+n*-e.z-r*-e.y,this.y=n*e.w+s*-e.y+r*-e.x-t*-e.z,this.z=r*e.w+s*-e.z+t*-e.y-n*-e.x},i.prototype.clone=function(){return new i(this)},rs=i,rs}var ss,co;function Ri(){if(co)return ss;co=1;function i(e){e=e||{},this.x=e.x||0,this.y=e.y||0,this.z=e.z||0,this.w=typeof e.w=="number"?e.w:1}return i.prototype.conjugate=function(){this.x*=-1,this.y*=-1,this.z*=-1},i.prototype.norm=function(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)},i.prototype.normalize=function(){var e=Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w);e===0?(this.x=0,this.y=0,this.z=0,this.w=1):(e=1/e,this.x=this.x*e,this.y=this.y*e,this.z=this.z*e,this.w=this.w*e)},i.prototype.invert=function(){this.conjugate(),this.normalize()},i.prototype.multiply=function(e){var t=this.x*e.w+this.y*e.z-this.z*e.y+this.w*e.x,n=-this.x*e.z+this.y*e.w+this.z*e.x+this.w*e.y,r=this.x*e.y-this.y*e.x+this.z*e.w+this.w*e.z,s=-this.x*e.x-this.y*e.y-this.z*e.z+this.w*e.w;this.x=t,this.y=n,this.z=r,this.w=s},i.prototype.clone=function(){return new i(this)},ss=i,ss}var as,uo;function Ma(){if(uo)return as;uo=1;var i=Fn(),e=Ri();function t(n){n=n||{},this.position=new i(n.position),this.orientation=new e(n.orientation)}return t.prototype.applyTransform=function(n){this.position.multiplyQuaternion(n.rotation),this.position.add(n.translation);var r=n.rotation.clone();r.multiply(this.orientation),this.orientation=r},t.prototype.clone=function(){return new t(this)},t.prototype.multiply=function(n){var r=n.clone();return r.applyTransform({rotation:this.orientation,translation:this.position}),r},t.prototype.getInverse=function(){var n=this.clone();return n.orientation.invert(),n.position.multiplyQuaternion(n.orientation),n.position.x*=-1,n.position.y*=-1,n.position.z*=-1,n},as=t,as}var os,ho;function yc(){if(ho)return os;ho=1;var i=Fn(),e=Ri();function t(n){n=n||{},this.translation=new i(n.translation),this.rotation=new e(n.rotation)}return t.prototype.clone=function(){return new t(this)},os=t,os}var ls,fo;function Xu(){return fo||(fo=1,ls={Pose:Ma(),Quaternion:Ri(),Transform:yc(),Vector3:Fn()}),ls}var cs={exports:{}},us,po;function qu(){if(po)return us;po=1;var i=vc(),e=xc(),t=br(),n=Ai(),r=Ci(),s=yc();function o(a){a=a||{},this.ros=a.ros,this.fixedFrame=a.fixedFrame||"base_link",this.angularThres=a.angularThres||2,this.transThres=a.transThres||.01,this.rate=a.rate||10,this.updateDelay=a.updateDelay||50;var l=a.topicTimeout||2,c=Math.floor(l),u=Math.floor((l-c)*1e9);this.topicTimeout={secs:c,nsecs:u},this.serverName=a.serverName||"/tf2_web_republisher",this.repubServiceName=a.repubServiceName||"/republish_tfs",this.currentGoal=!1,this.currentTopic=!1,this.frameInfos={},this.republisherUpdateRequested=!1,this._subscribeCB=null,this._isDisposed=!1,this.actionClient=new i({ros:a.ros,serverName:this.serverName,actionName:"tf2_web_republisher/TFSubscriptionAction",omitStatus:!0,omitResult:!0}),this.serviceClient=new t({ros:a.ros,name:this.repubServiceName,serviceType:"tf2_web_republisher/RepublishTFs"})}return o.prototype.processTFArray=function(a){a.transforms.forEach(function(l){var c=l.child_frame_id;c[0]==="/"&&(c=c.substring(1));var u=this.frameInfos[c];u&&(u.transform=new s({translation:l.transform.translation,rotation:l.transform.rotation}),u.cbs.forEach(function(h){h(u.transform)}))},this)},o.prototype.updateGoal=function(){var a={source_frames:Object.keys(this.frameInfos),target_frame:this.fixedFrame,angular_thres:this.angularThres,trans_thres:this.transThres,rate:this.rate};if(this.ros.groovyCompatibility)this.currentGoal&&this.currentGoal.cancel(),this.currentGoal=new e({actionClient:this.actionClient,goalMessage:a}),this.currentGoal.on("feedback",this.processTFArray.bind(this)),this.currentGoal.send();else{a.timeout=this.topicTimeout;var l=new n(a);this.serviceClient.callService(l,this.processResponse.bind(this))}this.republisherUpdateRequested=!1},o.prototype.processResponse=function(a){this._isDisposed||(this.currentTopic&&this.currentTopic.unsubscribe(this._subscribeCB),this.currentTopic=new r({ros:this.ros,name:a.topic_name,messageType:"tf2_web_republisher/TFArray"}),this._subscribeCB=this.processTFArray.bind(this),this.currentTopic.subscribe(this._subscribeCB))},o.prototype.subscribe=function(a,l){a[0]==="/"&&(a=a.substring(1)),this.frameInfos[a]?this.frameInfos[a].transform&&l(this.frameInfos[a].transform):(this.frameInfos[a]={cbs:[]},this.republisherUpdateRequested||(setTimeout(this.updateGoal.bind(this),this.updateDelay),this.republisherUpdateRequested=!0)),this.frameInfos[a].cbs.push(l)},o.prototype.unsubscribe=function(a,l){a[0]==="/"&&(a=a.substring(1));for(var c=this.frameInfos[a],u=c&&c.cbs||[],h=u.length;h--;)u[h]===l&&u.splice(h,1);(!l||u.length===0)&&delete this.frameInfos[a]},o.prototype.dispose=function(){this._isDisposed=!0,this.actionClient.dispose(),this.currentTopic&&this.currentTopic.unsubscribe(this._subscribeCB)},us=o,us}var mo;function Yu(){if(mo)return cs.exports;mo=1;var i=Sa(),e=ya(),t=cs.exports={TFClient:qu()};return e(i,["TFClient"],t),cs.exports}var hs,go;function Pi(){return go||(go=1,hs={URDF_SPHERE:0,URDF_BOX:1,URDF_CYLINDER:2,URDF_MESH:3}),hs}var ds,_o;function Sc(){if(_o)return ds;_o=1;var i=Fn(),e=Pi();function t(n){this.dimension=null,this.type=e.URDF_BOX;var r=n.xml.getAttribute("size").split(" ");this.dimension=new i({x:parseFloat(r[0]),y:parseFloat(r[1]),z:parseFloat(r[2])})}return ds=t,ds}var fs,vo;function Mc(){if(vo)return fs;vo=1;function i(e){var t=e.xml.getAttribute("rgba").split(" ");this.r=parseFloat(t[0]),this.g=parseFloat(t[1]),this.b=parseFloat(t[2]),this.a=parseFloat(t[3])}return fs=i,fs}var ps,xo;function bc(){if(xo)return ps;xo=1;var i=Pi();function e(t){this.type=i.URDF_CYLINDER,this.length=parseFloat(t.xml.getAttribute("length")),this.radius=parseFloat(t.xml.getAttribute("radius"))}return ps=e,ps}var ms,yo;function ba(){if(yo)return ms;yo=1;var i=Mc();function e(n){this.textureFilename=null,this.color=null,this.name=n.xml.getAttribute("name");var r=n.xml.getElementsByTagName("texture");r.length>0&&(this.textureFilename=r[0].getAttribute("filename"));var s=n.xml.getElementsByTagName("color");s.length>0&&(this.color=new i({xml:s[0]}))}e.prototype.isLink=function(){return this.color===null&&this.textureFilename===null};var t=Nn;return e.prototype.assign=function(n){return t(this,n)},ms=e,ms}var gs,So;function Ec(){if(So)return gs;So=1;var i=Fn(),e=Pi();function t(n){this.scale=null,this.type=e.URDF_MESH,this.filename=n.xml.getAttribute("filename");var r=n.xml.getAttribute("scale");if(r){var s=r.split(" ");this.scale=new i({x:parseFloat(s[0]),y:parseFloat(s[1]),z:parseFloat(s[2])})}}return gs=t,gs}var _s,Mo;function Tc(){if(Mo)return _s;Mo=1;var i=Pi();function e(t){this.type=i.URDF_SPHERE,this.radius=parseFloat(t.xml.getAttribute("radius"))}return _s=e,_s}var vs,bo;function wc(){if(bo)return vs;bo=1;var i=Ma(),e=Fn(),t=Ri(),n=bc(),r=Sc(),s=ba(),o=Ec(),a=Tc();function l(c){var u=c.xml;this.origin=null,this.geometry=null,this.material=null,this.name=c.xml.getAttribute("name");var h=u.getElementsByTagName("origin");if(h.length===0)this.origin=new i;else{var d=h[0].getAttribute("xyz"),f=new e;d&&(d=d.split(" "),f=new e({x:parseFloat(d[0]),y:parseFloat(d[1]),z:parseFloat(d[2])}));var g=h[0].getAttribute("rpy"),x=new t;if(g){g=g.split(" ");var m=parseFloat(g[0]),p=parseFloat(g[1]),T=parseFloat(g[2]),S=m/2,A=p/2,B=T/2,D=Math.sin(S)*Math.cos(A)*Math.cos(B)-Math.cos(S)*Math.sin(A)*Math.sin(B),P=Math.cos(S)*Math.sin(A)*Math.cos(B)+Math.sin(S)*Math.cos(A)*Math.sin(B),X=Math.cos(S)*Math.cos(A)*Math.sin(B)-Math.sin(S)*Math.sin(A)*Math.cos(B),E=Math.cos(S)*Math.cos(A)*Math.cos(B)+Math.sin(S)*Math.sin(A)*Math.sin(B);x=new t({x:D,y:P,z:X,w:E}),x.normalize()}this.origin=new i({position:f,orientation:x})}var M=u.getElementsByTagName("geometry");if(M.length>0){for(var C=M[0],H=null,O=0;O<C.childNodes.length;O++){var $=C.childNodes[O];if($.nodeType===1){H=$;break}}var V=H.nodeName;V==="sphere"?this.geometry=new a({xml:H}):V==="box"?this.geometry=new r({xml:H}):V==="cylinder"?this.geometry=new n({xml:H}):V==="mesh"?this.geometry=new o({xml:H}):console.warn("Unknown geometry type "+V)}var Y=u.getElementsByTagName("material");Y.length>0&&(this.material=new s({xml:Y[0]}))}return vs=l,vs}var xs,Eo;function Ac(){if(Eo)return xs;Eo=1;var i=wc();function e(t){this.name=t.xml.getAttribute("name"),this.visuals=[];for(var n=t.xml.getElementsByTagName("visual"),r=0;r<n.length;r++)this.visuals.push(new i({xml:n[r]}))}return xs=e,xs}var ys,To;function ju(){if(To)return ys;To=1;var i=Ma(),e=Fn(),t=Ri();function n(r){this.name=r.xml.getAttribute("name"),this.type=r.xml.getAttribute("type");var s=r.xml.getElementsByTagName("parent");s.length>0&&(this.parent=s[0].getAttribute("link"));var o=r.xml.getElementsByTagName("child");o.length>0&&(this.child=o[0].getAttribute("link"));var a=r.xml.getElementsByTagName("limit");a.length>0&&(this.minval=parseFloat(a[0].getAttribute("lower")),this.maxval=parseFloat(a[0].getAttribute("upper")));var l=r.xml.getElementsByTagName("origin");if(l.length===0)this.origin=new i;else{var c=l[0].getAttribute("xyz"),u=new e;c&&(c=c.split(" "),u=new e({x:parseFloat(c[0]),y:parseFloat(c[1]),z:parseFloat(c[2])}));var h=l[0].getAttribute("rpy"),d=new t;if(h){h=h.split(" ");var f=parseFloat(h[0]),g=parseFloat(h[1]),x=parseFloat(h[2]),m=f/2,p=g/2,T=x/2,S=Math.sin(m)*Math.cos(p)*Math.cos(T)-Math.cos(m)*Math.sin(p)*Math.sin(T),A=Math.cos(m)*Math.sin(p)*Math.cos(T)+Math.sin(m)*Math.cos(p)*Math.sin(T),B=Math.cos(m)*Math.cos(p)*Math.sin(T)-Math.sin(m)*Math.sin(p)*Math.cos(T),D=Math.cos(m)*Math.cos(p)*Math.cos(T)+Math.sin(m)*Math.sin(p)*Math.sin(T);d=new t({x:S,y:A,z:B,w:D}),d.normalize()}this.origin=new i({position:u,orientation:d})}}return ys=n,ys}var xi={},wo;function $u(){return wo||(wo=1,xi.DOMImplementation=window.DOMImplementation,xi.XMLSerializer=window.XMLSerializer,xi.DOMParser=window.DOMParser),xi}var Ss,Ao;function Ku(){if(Ao)return Ss;Ao=1;var i=ba(),e=Ac(),t=ju(),n=$u().DOMParser;function r(s){s=s||{};var o=s.xml,a=s.string;if(this.materials={},this.links={},this.joints={},a){var l=new n;o=l.parseFromString(a,"text/xml")}var c=o.documentElement;this.name=c.getAttribute("name");for(var u=c.childNodes,h=0;h<u.length;h++){var d=u[h];if(d.tagName==="material"){var f=new i({xml:d});this.materials[f.name]!==void 0?this.materials[f.name].isLink()?this.materials[f.name].assign(f):console.warn("Material "+f.name+"is not unique."):this.materials[f.name]=f}else if(d.tagName==="link"){var g=new e({xml:d});if(this.links[g.name]!==void 0)console.warn("Link "+g.name+" is not unique.");else{for(var x=0;x<g.visuals.length;x++){var m=g.visuals[x].material;m!==null&&m.name&&(this.materials[m.name]!==void 0?g.visuals[x].material=this.materials[m.name]:this.materials[m.name]=m)}this.links[g.name]=g}}else if(d.tagName==="joint"){var p=new t({xml:d});this.joints[p.name]=p}}}return Ss=r,Ss}var Ms,Co;function Zu(){return Co||(Co=1,Ms=Nn({UrdfBox:Sc(),UrdfColor:Mc(),UrdfCylinder:bc(),UrdfLink:Ac(),UrdfMaterial:ba(),UrdfMesh:Ec(),UrdfModel:Ku(),UrdfSphere:Tc(),UrdfVisual:wc()},Pi())),Ms}var _i=mc.ROSLIB||{REVISION:"1.4.1"},Li=Nn;Li(_i,Vu());Li(_i,Wu());Li(_i,Xu());Li(_i,Yu());Li(_i,Zu());var Ju=_i;const Ke=wu(Ju);function Qu(i,e){return i.id===17||i.id===16?{data:0}:i.id===5?{data:2}:i.id===18?{data:3}:i.id===19?{data:Number(e.globalZ)||0}:i.id===23?{data:Number(e.speed)||0}:i.id===24?{data:Number(e.fixedSpeed)||0}:{data:1}}function eh(i,e){return i.id===2?{position:{x:Number(e.startX)||0,y:Number(e.startY)||0,z:Number(e.startZ)||0},orientation:{x:Number(e.globalX)||0,y:Number(e.globalY)||0,z:Number(e.stepX)||0,w:Number(e.stepY)||0}}:i.id===6?{position:{x:Number(e.fixedX)||0,y:Number(e.fixedY)||0,z:Number(e.fixedZ)||0},orientation:{x:Number(e.fixedTheta)||0,y:0,z:0,w:1}}:i.id===7?{position:{x:Number(e.startX)||0,y:Number(e.startY)||0,z:Number(e.startZ)||0},orientation:{x:Number(e.speed)||0,y:0,z:0,w:1}}:i.id===22?{position:{x:Number(e.fixedX)||0,y:Number(e.fixedY)||0,z:Number(e.fixedZ)||0},orientation:{x:0,y:0,z:0,w:1}}:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}}class th{constructor({rosConnection:e,callbacks:t={}}){this.rosConnection=e,this.callbacks=t,this.publisherCache=new Map,this.toggleStates=new Map(Object.entries(hr()).map(([n,r])=>[n,r.value]))}reset(){this.publisherCache.clear(),this.toggleStates=new Map(Object.entries(hr()).map(([e,t])=>[e,t.value]))}getToggleStateSnapshot(){return Object.entries(hr()).reduce((e,[t,n])=>{var s,o,a,l;const r=this.toggleStates.has(t)?this.toggleStates.get(t):n.value;return e[t]={value:r,label:r?(s=cn(t))==null?void 0:s.activeLabel:(o=cn(t))==null?void 0:o.inactiveLabel,tone:r?(a=cn(t))==null?void 0:a.activeTone:(l=cn(t))==null?void 0:l.inactiveTone},e},{})}handle(e,t){var a,l,c,u,h,d,f,g,x,m,p,T;const n=ua.find(S=>S.id===e);if(!n){(l=(a=this.callbacks).onResultMessage)==null||l.call(a,`未识别的老前端命令: ${e}`),(u=(c=this.callbacks).onLog)==null||u.call(c,`未识别的老前端命令: ${e}`,"warn");return}const r=this.rosConnection.getResources();if(!(r!=null&&r.ros)){(d=(h=this.callbacks).onResultMessage)==null||d.call(h,`ROS 未连接，无法执行: ${n.name}`),(g=(f=this.callbacks).onLog)==null||g.call(f,`ROS 未连接，无法执行: ${n.name}`,"warn");return}const s=this.getOrCreatePublisher(r.ros,n.topic,n.type),o=this.buildMessagePayload(n,t);s.publish(new Ke.Message(o)),(m=(x=this.callbacks).onResultMessage)==null||m.call(x,`已发送 ${n.name} -> ${n.topic}`),(T=(p=this.callbacks).onLog)==null||T.call(p,`已发送 ${n.name} -> ${n.topic}`,"success")}handleToggle(e,t){var f,g,x,m,p,T,S,A,B,D,P,X,E,M,C,H;const n=cn(e);if(!n)return(g=(f=this.callbacks).onResultMessage)==null||g.call(f,`未识别的开关动作: ${e}`),(m=(x=this.callbacks).onLog)==null||m.call(x,`未识别的开关动作: ${e}`,"warn"),null;const r=this.rosConnection.getResources();if(!(r!=null&&r.ros))return(T=(p=this.callbacks).onResultMessage)==null||T.call(p,`ROS 未连接，无法执行: ${n.inactiveLabel}/${n.activeLabel}`),(A=(S=this.callbacks).onLog)==null||A.call(S,`ROS 未连接，无法执行开关: ${e}`,"warn"),null;const o=!(this.toggleStates.has(e)?this.toggleStates.get(e):!!n.initialValue),a=this.resolveToggleCommand(n,o);if(!a)return(D=(B=this.callbacks).onResultMessage)==null||D.call(B,`开关 ${e} 缺少对应命令`),(X=(P=this.callbacks).onLog)==null||X.call(P,`开关 ${e} 缺少对应命令`,"error"),null;const l=this.getOrCreatePublisher(r.ros,a.topic,a.type),c=this.buildToggleMessagePayload(n,a,o,t);l.publish(new Ke.Message(c)),this.toggleStates.set(e,o);const u=o?n.activeLabel:n.inactiveLabel,h=o?n.activeTone:n.inactiveTone,d=o?"已开启":"已关闭";return(M=(E=this.callbacks).onResultMessage)==null||M.call(E,`${u}，${d}`),(H=(C=this.callbacks).onLog)==null||H.call(C,`已发送 ${a.name} -> ${a.topic}，状态=${d}`,"success"),{value:o,label:u,tone:h}}getOrCreatePublisher(e,t,n){const r=`${t}|${n}`;if(this.publisherCache.has(r))return this.publisherCache.get(r);const s=new Ke.Topic({ros:e,name:t,messageType:n});return s.advertise(),this.publisherCache.set(r,s),s}buildMessagePayload(e,t){return e.type==="std_msgs/Float32"?Qu(e,t):e.type==="geometry_msgs/Pose"?eh(e,t):{}}resolveToggleCommand(e,t){const n=t?e.activateCommandId??e.commandId:e.deactivateCommandId??e.commandId;return ua.find(r=>r.id===n)||null}buildToggleMessagePayload(e,t,n,r){return e.messageType==="std_msgs/Bool"||t.type==="std_msgs/Bool"?{data:n}:this.buildMessagePayload(t,r)}}function nh({pathname:i="/",savedPreferences:e=null,currentHost:t="localhost"}={}){if(i&&i.length>1)try{const n=decodeURIComponent(i.substring(1));if(n.startsWith("ws://")||n.startsWith("wss://"))return n}catch{}return t?`ws://${t}:9090`:e!=null&&e.ip?`ws://${e.ip}:${e.port||"9090"}`:"ws://localhost:9090"}const ih="/Scepter/ir/image_raw",rh="/web/pointAI/set_workspace_quad",sh="/web/pointAI/run_workspace_s2",ah="/pointAI/manual_workspace_quad_pixels",oh="/pointAI/result_image_raw",lh="/Scepter/worldCoord/world_coord",ch="/Scepter/worldCoord/raw_world_coord",uh="/coordinate_point",hh="/cabin/pseudo_slam_markers",dh="/system_log/all",fh="/tf",ph="/tf_static",mh="/web/cabin/start_pseudo_slam_scan",gh="/web/cabin/start_global_work",_h="/web/cabin/run_bind_path_direct_test",vh="/cabin/set_execution_mode",xh="tie_robot_msgs/StartPseudoSlamScanTaskAction",yh="tie_robot_msgs/StartGlobalWorkTaskAction",Sh="tie_robot_msgs/RunBindPathDirectTestTaskAction",Mh="tie_robot_msgs/SetExecutionMode";class bh{constructor(e={}){this.callbacks=e,this.ros=null,this.resources=null,this.topicSubscribers=[]}connect(){var t,n,r,s;const e=nh({pathname:window.location.pathname,currentHost:window.location.hostname});(n=(t=this.callbacks).onConnectionInfo)==null||n.call(t,e,"连接中","info"),(s=(r=this.callbacks).onLog)==null||s.call(r,`开始连接 ROSBridge: ${e}`,"info"),this.ros=new Ke.Ros({url:e}),this.resources=this.buildResources(this.ros),this.ros.on("connection",()=>{var o,a,l,c,u,h;this.resources.workspaceQuadPublisher.advertise(),this.resources.runWorkspaceS2Publisher.advertise(),this.bindSubscriptions(),(a=(o=this.callbacks).onConnectionInfo)==null||a.call(o,e,"连接成功","success"),(c=(l=this.callbacks).onRosReady)==null||c.call(l,this.resources),(h=(u=this.callbacks).onLog)==null||h.call(u,"ROS 连接成功，动作链与图像订阅已就绪","success")}),this.ros.on("error",o=>{var l,c,u,h,d,f;const a=(o==null?void 0:o.message)||String(o);(c=(l=this.callbacks).onConnectionInfo)==null||c.call(l,e,"连接失败","error"),(h=(u=this.callbacks).onRosUnavailable)==null||h.call(u),(f=(d=this.callbacks).onLog)==null||f.call(d,`ROS 连接失败: ${a}`,"error")}),this.ros.on("close",()=>{var o,a,l,c,u,h;this.unbindSubscriptions(),(a=(o=this.callbacks).onConnectionInfo)==null||a.call(o,e,"连接失败","warn"),(c=(l=this.callbacks).onRosUnavailable)==null||c.call(l),(h=(u=this.callbacks).onLog)==null||h.call(u,"ROS 连接已断开","warn")})}buildResources(e){return{ros:e,workspaceQuadPublisher:new Ke.Topic({ros:e,name:rh,messageType:"std_msgs/Float32MultiArray"}),runWorkspaceS2Publisher:new Ke.Topic({ros:e,name:sh,messageType:"std_msgs/Bool"}),startPseudoSlamScanActionClient:new Ke.ActionClient({ros:e,serverName:mh,actionName:xh}),executionModeService:new Ke.Service({ros:e,name:vh,serviceType:Mh}),startGlobalWorkActionClient:new Ke.ActionClient({ros:e,serverName:gh,actionName:yh}),runDirectBindPathTestActionClient:new Ke.ActionClient({ros:e,serverName:_h,actionName:Sh})}}bindSubscriptions(){this.unbindSubscriptions();const e=[new Ke.Topic({ros:this.ros,name:ih,messageType:"sensor_msgs/Image"}),new Ke.Topic({ros:this.ros,name:ah,messageType:"std_msgs/Float32MultiArray"}),new Ke.Topic({ros:this.ros,name:oh,messageType:"sensor_msgs/Image"}),new Ke.Topic({ros:this.ros,name:lh,messageType:"sensor_msgs/Image"}),new Ke.Topic({ros:this.ros,name:ch,messageType:"sensor_msgs/Image"}),new Ke.Topic({ros:this.ros,name:uh,messageType:"tie_robot_msgs/PointsArray"}),new Ke.Topic({ros:this.ros,name:hh,messageType:"visualization_msgs/MarkerArray"}),new Ke.Topic({ros:this.ros,name:dh,messageType:"rosgraph_msgs/Log"}),new Ke.Topic({ros:this.ros,name:fh,messageType:"tf2_msgs/TFMessage"}),new Ke.Topic({ros:this.ros,name:ph,messageType:"tf2_msgs/TFMessage"})];e[0].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onBaseImage)==null?void 0:r.call(n,t)}),e[1].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onSavedWorkspacePayload)==null?void 0:r.call(n,Array.from(t.data||[]))}),e[2].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onExecutionOverlay)==null?void 0:r.call(n,t)}),e[3].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onPointCloudImage)==null?void 0:r.call(n,"filteredWorldCoord",t)}),e[4].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onPointCloudImage)==null?void 0:r.call(n,"rawWorldCoord",t)}),e[5].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onTiePoints)==null?void 0:r.call(n,t)}),e[6].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onPlanningMarkers)==null?void 0:r.call(n,t)}),e[7].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onSystemLog)==null?void 0:r.call(n,t)}),e[8].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onTfMessage)==null?void 0:r.call(n,t)}),e[9].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onTfMessage)==null?void 0:r.call(n,t)}),this.topicSubscribers=e}unbindSubscriptions(){this.topicSubscribers.forEach(e=>{try{e.unsubscribe()}catch{}}),this.topicSubscribers=[]}getResources(){return this.resources}isReady(){return!!(this.resources&&this.ros&&this.ros.isConnected)}}const Cc=[{id:"ros",label:"ROS",kind:"connection"},{id:"chassis",label:"索驱",topic:"/robot/chassis_status",messageType:"std_msgs/Float32"},{id:"moduan",label:"末端",topic:"/robot/moduan_status",messageType:"std_msgs/Float32"},{id:"bindingGun",label:"绑扎枪",topic:"/robot/binding_gun_status",messageType:"std_msgs/Float32"}],Eh="/moduan/moduan_gesture_data",Th="tie_robot_msgs/linear_module_upload";function wh(i){return Number.isFinite(i)?i<0?"error":i>0?"success":"info":"warn"}class Ah{constructor(e={}){this.callbacks=e,this.subscriptions=[],this.lastValues=new Map}setConnectionState(e,t){var n,r;(r=(n=this.callbacks).onStatusChip)==null||r.call(n,"ros",e,t)}start(e){this.stop(),Cc.filter(n=>n.kind!=="connection").forEach(n=>{const r=new Ke.Topic({ros:e,name:n.topic,messageType:n.messageType});r.subscribe(s=>{var c,u,h,d;const o=Number(s==null?void 0:s.data),a=wh(o),l=`${n.label}: ${Number.isFinite(o)?o:"无效值"}`;(u=(c=this.callbacks).onStatusChip)==null||u.call(c,n.id,a,l),this.lastValues.get(n.id)!==o&&((d=(h=this.callbacks).onLog)==null||d.call(h,`状态变化 ${n.label} -> ${l}`,a),this.lastValues.set(n.id,o))}),this.subscriptions.push(r)});const t=new Ke.Topic({ros:e,name:Eh,messageType:Th});t.subscribe(n=>{var s,o,a,l;const r=Number(n==null?void 0:n.robot_battery_voltage);if((o=(s=this.callbacks).onBatteryVoltage)==null||o.call(s,r),this.lastValues.get("robot_battery_voltage")!==r){const c=Number.isFinite(r)&&r>0?`机器人电压 ${r.toFixed(1)}V`:"机器人电压无效";(l=(a=this.callbacks).onLog)==null||l.call(a,`状态变化 电压 -> ${c}`,Number.isFinite(r)&&r>0?"info":"warn"),this.lastValues.set("robot_battery_voltage",r)}}),this.subscriptions.push(t)}stop(){var e,t;this.subscriptions.forEach(n=>{try{n.unsubscribe()}catch{}}),this.subscriptions=[],this.lastValues.delete("robot_battery_voltage"),(t=(e=this.callbacks).onBatteryVoltage)==null||t.call(e,Number.NaN)}}function Ch(i,e,t){return{x:Math.min(Math.max(Math.round(i.x),0),Math.max(e-1,0)),y:Math.min(Math.max(Math.round(i.y),0),Math.max(t-1,0))}}function bs({clientX:i,clientY:e,rect:t,imageWidth:n,imageHeight:r}){const s=Math.max((t==null?void 0:t.width)||0,1),o=Math.max((t==null?void 0:t.height)||0,1),a=(i-((t==null?void 0:t.left)||0))/s*n,l=(e-((t==null?void 0:t.top)||0))/o*r;return Ch({x:a,y:l},n,r)}function Rc(i){if(!Array.isArray(i)||i.length!==4)throw new Error("Workspace quad requires exactly four points");return i.flatMap(e=>[Math.round(e.x),Math.round(e.y)])}function Rh(i){if(!Array.isArray(i)||i.length!==8)throw new Error("Workspace quad payload requires exactly eight values");const e=[];for(let t=0;t<i.length;t+=2)e.push({x:Number(i[t]),y:Number(i[t+1])});return e}function Ph(i,e,t=12){if(!Array.isArray(i)||!e)return-1;let n=-1,r=Number.POSITIVE_INFINITY;const s=t*t;return i.forEach((o,a)=>{const l=Number(o.x)-Number(e.x),c=Number(o.y)-Number(e.y),u=l*l+c*c;u<=s&&u<r&&(r=u,n=a)}),n}function Lh(i,e,t){return(i||[]).map((n,r)=>r!==e?n:{x:Number(t.x),y:Number(t.y)})}function Pc(i){if(Array.isArray(i))return Uint8ClampedArray.from(i);if(typeof i=="string"){const e=atob(i),t=new Uint8ClampedArray(e.length);for(let n=0;n<e.length;n+=1)t[n]=e.charCodeAt(n);return t}return i instanceof Uint8Array||i instanceof Uint8ClampedArray?new Uint8ClampedArray(i):(i==null?void 0:i.buffer)instanceof ArrayBuffer?new Uint8ClampedArray(i.buffer):null}function Ro(i,e){const t=new Uint32Array(256);for(let o=0;o<i.length;o+=1)t[i[o]]+=1;const n=Math.min(Math.max(e,0),100),r=Math.max(Math.ceil(n/100*i.length),1);let s=0;for(let o=0;o<t.length;o+=1)if(s+=t[o],s>=r)return o;return 255}function Uh(i,{lowClipPercent:e=1,highClipPercent:t=99}={}){const n=Ro(i,e),r=Ro(i,t);if(r<=n)return Uint8ClampedArray.from(i);const s=new Uint8ClampedArray(i.length),o=255/(r-n);for(let a=0;a<i.length;a+=1){const l=Math.round((i[a]-n)*o);s[a]=Math.min(Math.max(l,0),255)}return s}function Dh(i,e=1){const t=Number.isFinite(e)&&e>0?e:1,n=new Uint8ClampedArray(i.length);for(let r=0;r<i.length;r+=1)n[r]=Math.round(255*(i[r]/255)**t);return n}function Ih(i){const e=new Uint32Array(256);for(let a=0;a<i.length;a+=1)e[i[a]]+=1;let t=0;const n=new Uint32Array(256);let r=0;for(let a=0;a<e.length;a+=1)r+=e[a],n[a]=r,t===0&&e[a]>0&&(t=r);if(t===i.length)return Uint8ClampedArray.from(i);const s=new Uint8ClampedArray(i.length),o=Math.max(i.length-t,1);for(let a=0;a<i.length;a+=1)s[a]=Math.round((n[i[a]]-t)/o*255);return s}function Nh(i,{mode:e="auto",gamma:t=.85}={}){if(e==="raw")return Uint8ClampedArray.from(i);let n=Uh(i);return e==="strong"&&(n=Ih(n)),Dh(n,t)}function Po(i,e){const t=Pc(i.data),n=Number(i.width)||0,r=Number(i.height)||0,s=String(i.encoding||"").toLowerCase();if(!t||n<=0||r<=0)throw new Error("图像数据无效");const o=new Uint8ClampedArray(n*r*4);if(s.includes("mono8")||s.includes("8uc1")){const a=Nh(t,e);for(let l=0;l<n*r;l+=1){const c=a[l]??0,u=l*4;o[u]=c,o[u+1]=c,o[u+2]=c,o[u+3]=255}return new ImageData(o,n,r)}if(s.includes("rgb8")||s.includes("bgr8")){for(let a=0;a<n*r;a+=1){const l=a*3,c=a*4,u=s.includes("bgr8")?[t[l+2],t[l+1],t[l]]:[t[l],t[l+1],t[l+2]];o[c]=u[0]??0,o[c+1]=u[1]??0,o[c+2]=u[2]??0,o[c+3]=255}return new ImageData(o,n,r)}throw new Error(`暂不支持的图像编码: ${i.encoding}`)}function Oh(i,{sampleStep:e=4,maxPoints:t=24e3}={}){const n=Pc(i==null?void 0:i.data),r=Number(i==null?void 0:i.width)||0,s=Number(i==null?void 0:i.height)||0;if(!n||r<=0||s<=0)return{positions:new Float32Array,count:0};const o=new DataView(n.buffer,n.byteOffset,n.byteLength),a=Math.max(1,Number(e)||1),l=[];let c=0;for(let u=0;u<s;u+=a)for(let h=0;h<r;h+=a){const d=(u*r+h)*12;if(d+12>o.byteLength)continue;const f=o.getFloat32(d,!0),g=o.getFloat32(d+4,!0),x=o.getFloat32(d+8,!0);if(!(!Number.isFinite(f)||!Number.isFinite(g)||!Number.isFinite(x))&&!(f===0&&g===0&&x===0)&&(l.push(f/1e3,g/1e3,x/1e3),c+=1,c>=t))return{positions:Float32Array.from(l),count:c}}return{positions:Float32Array.from(l),count:c}}class Fh{constructor({rosConnection:e,workspaceView:t,callbacks:n={}}){this.rosConnection=e,this.workspaceView=t,this.callbacks=n}handle(e){switch(e){case"submitQuad":return this.publishWorkspaceQuad();case"runSavedS2":return this.triggerSavedWorkspaceS2();case"scanPlan":return this.triggerPseudoSlamScan();case"startExecution":return this.triggerExecutionLayer(!0);case"startExecutionKeepMemory":return this.triggerExecutionLayer(!1);case"runBindPathTest":return this.triggerBindPathDirectTest();default:this.report(`未识别的任务动作: ${e}`,"warn")}}publishWorkspaceQuad(){var r,s,o,a;const e=this.rosConnection.getResources(),t=this.workspaceView.getSelectedPoints();if(!(e!=null&&e.workspaceQuadPublisher)||!(e!=null&&e.runWorkspaceS2Publisher)||t.length!==4){this.report("当前还不能提交工作区四边形，请先连上 ROS 并点满 4 个角点","warn");return}const n=Rc(t);this.workspaceView.setExecutionOverlayMessage(null),e.workspaceQuadPublisher.publish(new Ke.Message({data:n})),(s=(r=this.callbacks).onResultMessage)==null||s.call(r,`工作区四边形已发送，正在触发 S2，覆盖层统一等待 result_img: [${n.join(", ")}]`),(a=(o=this.callbacks).onLog)==null||a.call(o,`已发送工作区四边形: [${n.join(", ")}]`,"success"),window.setTimeout(()=>{e.runWorkspaceS2Publisher.publish(new Ke.Message({data:!0}))},180)}triggerSavedWorkspaceS2(){var n,r,s,o;const e=this.rosConnection.getResources(),t=this.workspaceView.getSavedWorkspacePoints();if(!(e!=null&&e.runWorkspaceS2Publisher)||t.length!==4){this.report("当前没有可复用的已保存工作区，请先点 4 个角点并提交","warn");return}this.workspaceView.setExecutionOverlayMessage(null),e.runWorkspaceS2Publisher.publish(new Ke.Message({data:!0})),(r=(n=this.callbacks).onResultMessage)==null||r.call(n,"正在使用当前已保存工作区识别绑扎点，覆盖层统一等待 result_img..."),(o=(s=this.callbacks).onLog)==null||o.call(s,"已触发基于已保存工作区的 S2 识别","success")}triggerPseudoSlamScan(){var t,n,r,s;const e=this.rosConnection.getResources();if(!(e!=null&&e.startPseudoSlamScanActionClient)){this.report("ROS 还没连好，暂时不能开始固定扫描规划","warn");return}this.workspaceView.setExecutionOverlayMessage(null),(n=(t=this.callbacks).onResultMessage)==null||n.call(t,"正在执行固定工作区扫描：移动到 x=-260, y=1700, z=2997, speed=100，然后触发 S2 并动态规划，覆盖层统一显示 result_img。"),(s=(r=this.callbacks).onLog)==null||s.call(r,"已触发固定扫描建图任务","success"),this.sendActionGoal(e.startPseudoSlamScanActionClient,{goalMessage:{enable_capture_gate:!1,scan_strategy:2},feedbackPrefix:"扫描建图进行中",successPrefix:"扫描建图完成",failurePrefix:"扫描建图失败"})}triggerExecutionLayer(e){var r,s,o,a;const t=this.rosConnection.getResources();if(!(t!=null&&t.executionModeService)||!(t!=null&&t.startGlobalWorkActionClient)){this.report("ROS 还没连好，暂时不能开始执行层","warn");return}this.workspaceView.setExecutionOverlayMessage(null),(s=(r=this.callbacks).onResultMessage)==null||s.call(r,"工作区覆盖层统一使用 result_img，后续显示 /pointAI/result_image_raw。"),(a=(o=this.callbacks).onLog)==null||a.call(o,e?"准备清记忆并开始执行层":"准备保留记忆直接开始执行层","success");const n=new Ke.ServiceRequest({execution_mode:1});t.executionModeService.callService(n,l=>{if(!(l!=null&&l.success)){this.report(`执行模式切换失败: ${(l==null?void 0:l.message)||"未知错误"}`,"error");return}this.sendActionGoal(t.startGlobalWorkActionClient,{goalMessage:{clear_execution_memory:e,execution_mode:1},feedbackPrefix:"执行层进行中",successPrefix:"执行层任务完成",failurePrefix:"执行层任务失败"})},l=>this.report(`执行模式切换失败: ${(l==null?void 0:l.message)||String(l)}`,"error"))}triggerBindPathDirectTest(){var t,n,r,s;const e=this.rosConnection.getResources();if(!(e!=null&&e.runDirectBindPathTestActionClient)){this.report("ROS 还没连好，暂时不能直接执行账本测试","warn");return}(n=(t=this.callbacks).onResultMessage)==null||n.call(t,"直接执行账本测试已触发：后端将只按 pseudo_slam_bind_path.json 的 path_origin、cabin_pose 和 x/y/z 执行。"),(s=(r=this.callbacks).onLog)==null||s.call(r,"已触发直接执行账本测试","success"),this.sendActionGoal(e.runDirectBindPathTestActionClient,{goalMessage:{},feedbackPrefix:"账本测试进行中",successPrefix:"账本测试完成",failurePrefix:"账本测试失败"})}sendActionGoal(e,{goalMessage:t,feedbackPrefix:n,successPrefix:r,failurePrefix:s}){const o=new Ke.Goal({actionClient:e,goalMessage:t});o.on("feedback",a=>{var c,u;const l=(a==null?void 0:a.detail)||(a==null?void 0:a.stage)||"处理中";(u=(c=this.callbacks).onResultMessage)==null||u.call(c,`${n}: ${l}`)}),o.on("result",a=>{var c,u,h,d,f,g,x,m;const l=(a==null?void 0:a.message)||"未知结果";if(a!=null&&a.success){(u=(c=this.callbacks).onResultMessage)==null||u.call(c,`${r}: ${l}`),(d=(h=this.callbacks).onLog)==null||d.call(h,`${r}: ${l}`,"success");return}(g=(f=this.callbacks).onResultMessage)==null||g.call(f,`${s}: ${l}`),(m=(x=this.callbacks).onLog)==null||m.call(x,`${s}: ${l}`,"error")}),o.send()}report(e,t="info"){var n,r,s,o;(r=(n=this.callbacks).onResultMessage)==null||r.call(n,e),(o=(s=this.callbacks).onLog)==null||o.call(s,e,t)}}const Lc=[{id:"onlyPointCloud",label:"只显示点云"},{id:"onlyTiePoints",label:"只显示绑扎点"},{id:"pointCloudAndTiePoints",label:"点云 + 绑扎点"},{id:"planningFocus",label:"规划点/执行点"},{id:"machineOnly",label:"只显示机器"},{id:"all",label:"全开"}],Uc=[{id:"filteredWorldCoord",label:"滤波世界点云"},{id:"rawWorldCoord",label:"原始世界点云"}],Dc=[{id:"camera",label:"相机视角"},{id:"global",label:"全局视角"}];function Ea(i,e,t=e){var n;return((n=i.find(r=>r.id===e))==null?void 0:n.label)||t}const Lo={onlyPointCloud:{showRobot:!1,showAxes:!1,showPointCloud:!0,showTiePoints:!1,showPlanningMarkers:!1},onlyTiePoints:{showRobot:!1,showAxes:!0,showPointCloud:!1,showTiePoints:!0,showPlanningMarkers:!1},pointCloudAndTiePoints:{showRobot:!0,showAxes:!0,showPointCloud:!0,showTiePoints:!0,showPlanningMarkers:!1},planningFocus:{showRobot:!0,showAxes:!0,showPointCloud:!1,showTiePoints:!1,showPlanningMarkers:!0},machineOnly:{showRobot:!0,showAxes:!0,showPointCloud:!1,showTiePoints:!1,showPlanningMarkers:!1},all:{showRobot:!0,showAxes:!0,showPointCloud:!0,showTiePoints:!0,showPlanningMarkers:!0}},Bh={mode:"pointCloudAndTiePoints",pointCloudSource:"filteredWorldCoord",showRobot:!0,showAxes:!0,showPointCloud:!0,showTiePoints:!0,showPlanningMarkers:!1,pointSize:.035,pointOpacity:.78,viewMode:"camera",followCamera:!1};function kh(i,e){const t=Lo[i]||Lo.pointCloudAndTiePoints;return{...e,mode:i,...t}}function Ic(i){return Ea(Lc,i)}function zh(i){return Ea(Uc,i)}function Vh(i){return Ea(Dc,i)}class Gh{constructor({ui:e,sceneView:t,callbacks:n={}}){this.ui=e,this.sceneView=t,this.callbacks=n,this.state={...Bh},this.stats={filteredWorldCoordCount:0,rawWorldCoordCount:0,tiePointCount:0,planningPointCount:0,tfFrameCount:0}}init(){this.ui.setTopicLayerState(this.state),this.ui.renderTopicLayerStats(this.stats),this.sceneView.setLayerState(this.state),this.sceneView.setViewMode(this.state.viewMode),this.sceneView.setFollowCamera(this.state.followCamera)}handleLayerControlsChange(e){var r,s;const t=e.mode||this.state.mode;let n={...this.state,...e};e.mode&&e.mode!==this.state.mode&&(n=kh(t,n)),this.state=n,this.ui.setTopicLayerState(this.state),this.sceneView.setLayerState(this.state),(s=(r=this.callbacks).onLog)==null||s.call(r,`三维图层模式已切换为 ${Ic(this.state.mode)}`,"info")}handleSceneControlsChange(e){var t,n;this.state={...this.state,...e},this.ui.setTopicLayerState(this.state),this.sceneView.setViewMode(this.state.viewMode),this.sceneView.setFollowCamera(this.state.followCamera),(n=(t=this.callbacks).onLog)==null||n.call(t,`三维视角已切换为 ${Vh(this.state.viewMode)}${this.state.followCamera?"（跟随相机）":""}`,"info")}updateStats(e){this.stats={...this.stats,...e},this.ui.renderTopicLayerStats(this.stats)}getState(){return{...this.state}}}class Hh{constructor(){this.panels=new Map}registerPanel(e){if(!e)return;const t=e.querySelector(".panel-header");if(!t)return;let n=!1,r=0,s=0,o=0,a=0;const l=()=>{let h=20;this.panels.forEach(d=>{h=Math.max(h,Number(d.style.zIndex||20))}),e.style.zIndex=String(h+1)},c=h=>{if(!n)return;const d=h.clientX-o,f=h.clientY-a;e.style.left=`${r+d}px`,e.style.top=`${s+f}px`,e.style.right="auto"},u=()=>{n=!1,document.removeEventListener("pointermove",c),document.removeEventListener("pointerup",u)};t.addEventListener("pointerdown",h=>{if(window.innerWidth<=1100||h.target.closest("button, a, input, select, label"))return;n=!0,l();const f=e.getBoundingClientRect();r=f.left,s=f.top,o=h.clientX,a=h.clientY,e.style.left=`${f.left}px`,e.style.top=`${f.top}px`,e.style.right="auto",document.addEventListener("pointermove",c),document.addEventListener("pointerup",u)}),this.panels.set(e.id,e)}init(e){e.querySelectorAll(".floating-panel").forEach(t=>this.registerPanel(t))}}const Uo=[{id:"controlPanel",label:"控制面板"},{id:"workspacePanel",label:"工作区"},{id:"topicLayersPanel",label:"话题图层"},{id:"logPanel",label:"日志"}],Wh={controlPanel:!0,workspacePanel:!0,topicLayersPanel:!0,logPanel:!1},Xh={auto:"自动增强",strong:"强增强",raw:"原图"},Es={info:"连接中",success:"连接成功",warn:"连接失败",error:"连接失败"};class qh{constructor(e){this.rootElement=e,this.refs={}}renderShell(){this.rootElement.innerHTML=`
      <div class="app-shell">
        <div id="sceneBackground" class="scene-background"></div>
        <div class="scene-overlay"></div>

        <header class="top-toolbar">
          <div class="toolbar-group toolbar-brand-group">
            <div class="toolbar-brand">绑扎机器人</div>
          </div>

          <div class="toolbar-group">
            ${Uo.map(e=>`
              <button
                class="toolbar-pill active"
                type="button"
                data-toolbar-action="toggle-panel:${e.id}"
                data-panel-toggle="${e.id}"
              >${e.label}</button>
            `).join("")}
          </div>

          <div class="toolbar-group toolbar-status-group">
            <div id="connectionBadge" class="toolbar-connection-badge info" title="连接中">连接中</div>
            <div id="voltageBadge" class="toolbar-voltage-badge" title="机器人电压暂未获取">电压 --.-V</div>
            <div id="statusChips" class="toolbar-status-chips"></div>
          </div>

          <div class="toolbar-group toolbar-link-group">
            <a class="toolbar-pill toolbar-link" href="/help/" target="_blank" rel="noreferrer">帮助</a>
          </div>
        </header>

        <div class="floating-panels">
          <section id="controlPanel" class="floating-panel panel-control" data-size="wide">
            <div class="panel-header">
              <div>
                <div class="panel-title">控制面板</div>
                <div class="panel-subtitle">快速排查、调试与执行入口</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:controlPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content control-panel-content">
              <div class="section-title control-section-title">主链任务</div>
              <div id="controlPanelTaskGrid" class="control-button-grid"></div>

              <div class="section-title control-section-title">调试与排查</div>
              <div id="controlPanelGroups" class="control-group-stack"></div>
            </div>
          </section>

          <section id="workspacePanel" class="floating-panel panel-workspace">
            <div class="panel-header">
              <div>
                <div class="panel-title">工作区</div>
                <div class="panel-subtitle">IR 工作区选点与结果叠加</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:workspacePanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content workspace-panel-content">
              <div class="canvas-stage">
                <canvas id="irCanvas" width="640" height="480"></canvas>
                <canvas id="overlayCanvas" width="640" height="480"></canvas>
              </div>

              <div class="workspace-metadata">
                <div class="section-title">已选角点</div>
                <ol id="selectedPoints" class="point-list"></ol>
                <div class="button-row">
                  <button class="secondary-btn" data-workspace-action="undo" disabled>撤销最后一点</button>
                  <button class="secondary-btn" data-workspace-action="clear" disabled>清空重选</button>
                </div>
              </div>

              <div class="section-title">显示增强</div>
              <div class="field-grid single-column">
                <div class="field">
                  <label for="displayMode">模式</label>
                  <select id="displayMode">
                    <option value="auto">自动增强</option>
                    <option value="strong">强增强</option>
                    <option value="raw">原图</option>
                  </select>
                </div>
                <div class="field">
                  <label for="gammaRange">伽马</label>
                  <input id="gammaRange" type="range" min="0.40" max="1.60" step="0.05" value="0.85" />
                </div>
                <div class="field">
                  <label for="overlayOpacityRange">覆盖透明度</label>
                  <input id="overlayOpacityRange" type="range" min="0.00" max="1.00" step="0.02" value="0.88" />
                </div>
                <div class="info-block mono" id="displaySettingSummary">模式=自动增强 伽马=0.85 覆盖=0.88</div>
              </div>
            </div>
          </section>

          <section id="topicLayersPanel" class="floating-panel panel-topic-layers">
            <div class="panel-header">
              <div>
                <div class="panel-title">话题图层</div>
                <div class="panel-subtitle">按相机模态、TF 与话题切换图层</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:topicLayersPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <div class="section-title">场景视角</div>
              <div class="field-grid compact-grid">
                <div class="field">
                  <label for="sceneViewMode">视角</label>
                  <select id="sceneViewMode">
                    ${Dc.map(e=>`
                      <option value="${e.id}">${e.label}</option>
                    `).join("")}
                  </select>
                </div>
                <label class="checkbox-field">
                  <input id="followCameraToggle" type="checkbox" />
                  <span>跟随相机</span>
                </label>
              </div>

              <div class="section-title">显示模式</div>
              <div class="field-grid compact-grid">
                <div class="field">
                  <label for="topicLayerMode">模式</label>
                  <select id="topicLayerMode">
                    ${Lc.map(e=>`
                      <option value="${e.id}">${e.label}</option>
                    `).join("")}
                  </select>
                </div>
                <div class="field">
                  <label for="pointCloudSource">点云源</label>
                  <select id="pointCloudSource">
                    ${Uc.map(e=>`
                      <option value="${e.id}">${e.label}</option>
                    `).join("")}
                  </select>
                </div>
              </div>

              <div class="toggle-grid">
                <label class="checkbox-field"><input id="showRobotToggle" type="checkbox" checked /><span>机器</span></label>
                <label class="checkbox-field"><input id="showAxesToggle" type="checkbox" checked /><span>坐标轴</span></label>
                <label class="checkbox-field"><input id="showPointCloudToggle" type="checkbox" checked /><span>点云</span></label>
                <label class="checkbox-field"><input id="showTiePointsToggle" type="checkbox" checked /><span>绑扎点</span></label>
                <label class="checkbox-field"><input id="showPlanningMarkersToggle" type="checkbox" /><span>规划点</span></label>
              </div>

              <div class="field-grid compact-grid" style="margin-top: 12px;">
                <div class="field">
                  <label for="pointSizeRange">点大小</label>
                  <input id="pointSizeRange" type="range" min="0.01" max="0.10" step="0.005" value="0.035" />
                </div>
                <div class="field">
                  <label for="pointOpacityRange">点透明度</label>
                  <input id="pointOpacityRange" type="range" min="0.15" max="1.00" step="0.05" value="0.78" />
                </div>
              </div>

              <div id="topicLayerSummary" class="info-block mono">模式=点云 + 绑扎点 点云源=滤波世界点云 点大小=0.035 透明度=0.78</div>

              <div class="section-title">当前数据量</div>
              <div id="topicLayerStats" class="stats-grid"></div>
            </div>
          </section>

          <section id="logPanel" class="floating-panel panel-log">
            <div class="panel-header">
              <div>
              <div class="panel-title">日志</div>
                <div class="panel-subtitle">终端日志订阅</div>
              </div>
              <div class="panel-actions">
                <button id="clearLogs" class="panel-action-btn" type="button" title="清空日志">⌫</button>
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:logPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <ul id="logList" class="log-list"></ul>
            </div>
          </section>
        </div>
      </div>
    `,this.bindRefs(),this.applyDefaultPanelVisibility(),this.renderStatusChips(),this.renderControlPanelTasks(),this.renderLegacyCommands(),this.renderPointList([]),this.renderLogs([]),this.renderTopicLayerStats({filteredWorldCoordCount:0,rawWorldCoordCount:0,tiePointCount:0,planningPointCount:0,tfFrameCount:0})}bindRefs(){this.refs.sceneBackground=this.rootElement.querySelector("#sceneBackground"),this.refs.irCanvas=this.rootElement.querySelector("#irCanvas"),this.refs.overlayCanvas=this.rootElement.querySelector("#overlayCanvas"),this.refs.sceneViewMode=this.rootElement.querySelector("#sceneViewMode"),this.refs.followCameraToggle=this.rootElement.querySelector("#followCameraToggle"),this.refs.connectionBadge=this.rootElement.querySelector("#connectionBadge"),this.refs.voltageBadge=this.rootElement.querySelector("#voltageBadge"),this.refs.controlPanelTaskGrid=this.rootElement.querySelector("#controlPanelTaskGrid"),this.refs.controlPanelGroups=this.rootElement.querySelector("#controlPanelGroups"),this.refs.topicLayerMode=this.rootElement.querySelector("#topicLayerMode"),this.refs.pointCloudSource=this.rootElement.querySelector("#pointCloudSource"),this.refs.showRobotToggle=this.rootElement.querySelector("#showRobotToggle"),this.refs.showAxesToggle=this.rootElement.querySelector("#showAxesToggle"),this.refs.showPointCloudToggle=this.rootElement.querySelector("#showPointCloudToggle"),this.refs.showTiePointsToggle=this.rootElement.querySelector("#showTiePointsToggle"),this.refs.showPlanningMarkersToggle=this.rootElement.querySelector("#showPlanningMarkersToggle"),this.refs.pointSizeRange=this.rootElement.querySelector("#pointSizeRange"),this.refs.pointOpacityRange=this.rootElement.querySelector("#pointOpacityRange"),this.refs.topicLayerSummary=this.rootElement.querySelector("#topicLayerSummary"),this.refs.topicLayerStats=this.rootElement.querySelector("#topicLayerStats"),this.refs.selectedPoints=this.rootElement.querySelector("#selectedPoints"),this.refs.displayMode=this.rootElement.querySelector("#displayMode"),this.refs.gammaRange=this.rootElement.querySelector("#gammaRange"),this.refs.overlayOpacityRange=this.rootElement.querySelector("#overlayOpacityRange"),this.refs.displaySettingSummary=this.rootElement.querySelector("#displaySettingSummary"),this.refs.statusChips=this.rootElement.querySelector("#statusChips"),this.refs.logList=this.rootElement.querySelector("#logList"),this.refs.clearLogs=this.rootElement.querySelector("#clearLogs"),this.refs.taskButtons=[...this.rootElement.querySelectorAll("[data-task-action]")],this.refs.workspaceButtons=[...this.rootElement.querySelectorAll("[data-workspace-action]")],this.refs.toolbarButtons=[...this.rootElement.querySelectorAll("[data-toolbar-action]")],this.refs.panelElements=new Map(Uo.map(({id:e})=>[e,this.rootElement.querySelector(`#${e}`)])),this.refs.panelToggleButtons=[...this.rootElement.querySelectorAll("[data-panel-toggle]")],this.refs.topicLayerInputs=[this.refs.topicLayerMode,this.refs.pointCloudSource,this.refs.showRobotToggle,this.refs.showAxesToggle,this.refs.showPointCloudToggle,this.refs.showTiePointsToggle,this.refs.showPlanningMarkersToggle,this.refs.pointSizeRange,this.refs.pointOpacityRange],this.refs.sceneInputs=[this.refs.sceneViewMode,this.refs.followCameraToggle]}applyDefaultPanelVisibility(){Object.entries(Wh).forEach(([e,t])=>{this.setPanelVisible(e,t)})}renderStatusChips(){this.refs.statusChips.innerHTML=Cc.map(e=>`
      <span class="status-chip info" data-status-id="${e.id}" title="${e.label}">
        <span>${e.label}</span>
      </span>
    `).join("")}renderControlPanelTasks(){this.refs.controlPanelTaskGrid.innerHTML=Mu.map(e=>`
      <button
        class="control-action-btn"
        type="button"
        data-task-action="${e.id}"
        data-tone="${e.tone}"
        disabled
      >${e.label.replaceAll(`
`,"<br />")}</button>
    `).join(""),this.refs.taskButtons=[...this.refs.controlPanelTaskGrid.querySelectorAll("[data-task-action]")]}renderLegacyCommands(){this.refs.controlPanelGroups.innerHTML=Tu().map(({title:e,controls:t})=>`
      <div class="control-command-group">
        <div class="control-group-title">${e}</div>
        <div class="control-button-grid">
          ${t.map(n=>`
            ${n.kind==="toggle"?`
              <button
                class="control-action-btn ${n.active?"is-active":""}"
                data-control-toggle="${n.id}"
                data-tone="${n.tone}"
                data-active="${n.active?"true":"false"}"
                type="button"
              >
                ${n.label}
              </button>
            `:`
              <button
                class="control-action-btn"
                data-legacy-command="${n.commandId}"
                data-tone="${fc(n.commandId)}"
                type="button"
              >
                ${n.label}
              </button>
            `}
          `).join("")}
        </div>
      </div>
    `).join(""),this.refs.legacyButtons=[...this.rootElement.querySelectorAll("[data-legacy-command]")],this.refs.toggleButtons=[...this.rootElement.querySelectorAll("[data-control-toggle]")]}getCanvasRefs(){return{canvas:this.refs.irCanvas,overlayCanvas:this.refs.overlayCanvas}}getSceneContainer(){return this.refs.sceneBackground}getDisplayControls(){return{displayMode:this.refs.displayMode,gammaRange:this.refs.gammaRange,overlayOpacityRange:this.refs.overlayOpacityRange}}getParameterValues(){return Object.keys(Ia).reduce((e,t)=>{const n=this.rootElement.querySelector(`#param-${t}`);return e[t]=Number.parseFloat((n==null?void 0:n.value)??Ia[t]),e},{})}getTopicLayerState(){return{mode:this.refs.topicLayerMode.value,pointCloudSource:this.refs.pointCloudSource.value,showRobot:this.refs.showRobotToggle.checked,showAxes:this.refs.showAxesToggle.checked,showPointCloud:this.refs.showPointCloudToggle.checked,showTiePoints:this.refs.showTiePointsToggle.checked,showPlanningMarkers:this.refs.showPlanningMarkersToggle.checked,pointSize:Number.parseFloat(this.refs.pointSizeRange.value),pointOpacity:Number.parseFloat(this.refs.pointOpacityRange.value),viewMode:this.refs.sceneViewMode.value,followCamera:this.refs.followCameraToggle.checked}}getSceneViewState(){return{viewMode:this.refs.sceneViewMode.value,followCamera:this.refs.followCameraToggle.checked}}isPanelVisible(e){const t=this.refs.panelElements.get(e);return!!(t&&t.dataset.visible!=="false")}setPanelVisible(e,t){const n=this.refs.panelElements.get(e);n&&(n.hidden=!t,n.dataset.visible=t?"true":"false",n.style.display=t?"":"none",this.refs.panelToggleButtons.filter(r=>r.dataset.panelToggle===e).forEach(r=>{r.classList.toggle("active",t),r.setAttribute("aria-pressed",t?"true":"false")}))}togglePanelVisible(e){const t=!this.isPanelVisible(e);return this.setPanelVisible(e,t),t}onToolbarAction(e){this.refs.toolbarButtons.forEach(t=>{t.addEventListener("click",n=>{n.preventDefault(),n.stopPropagation(),e(t.dataset.toolbarAction)})})}onTaskAction(e){this.refs.taskButtons.forEach(t=>{t.addEventListener("click",()=>e(t.dataset.taskAction))})}onWorkspaceAction(e){this.refs.workspaceButtons.forEach(t=>{t.addEventListener("click",()=>e(t.dataset.workspaceAction))})}onLegacyCommand(e){this.refs.legacyButtons.forEach(t=>{t.addEventListener("click",()=>e(Number(t.dataset.legacyCommand)))})}onControlToggle(e){this.refs.toggleButtons.forEach(t=>{t.addEventListener("click",()=>e(t.dataset.controlToggle))})}onDisplaySettingsChange(e){[this.refs.displayMode,this.refs.gammaRange,this.refs.overlayOpacityRange].forEach(t=>{t.addEventListener("input",()=>e(this.getDisplaySettings())),t.addEventListener("change",()=>e(this.getDisplaySettings()))})}onSceneControlsChange(e){this.refs.sceneInputs.forEach(t=>{const n=t.type==="checkbox"?"change":"input";t.addEventListener(n,()=>e(this.getSceneViewState()))})}onTopicLayerControlsChange(e){this.refs.topicLayerInputs.forEach(t=>{const n=t.type==="checkbox"?"change":"input";t.addEventListener(n,()=>e(this.getTopicLayerState())),n!=="change"&&t.addEventListener("change",()=>e(this.getTopicLayerState()))})}onClearLogs(e){this.refs.clearLogs.addEventListener("click",e)}setConnectionInfo(e,t,n="info"){const r=Es[n]?n:"info";this.refs.connectionBadge.textContent=Es[r],this.refs.connectionBadge.className=`toolbar-connection-badge ${r}`,this.refs.connectionBadge.title=t||e||Es[r]}setBatteryVoltage(e){if(!Number.isFinite(e)||e<=0||e>100){this.refs.voltageBadge.textContent="电压 --.-V",this.refs.voltageBadge.title="机器人电压无效或超出显示范围",this.refs.voltageBadge.className="toolbar-voltage-badge warn";return}const t=Number(e).toFixed(1);this.refs.voltageBadge.textContent=`电压 ${t}V`,this.refs.voltageBadge.title=`机器人电压 ${t}V`,this.refs.voltageBadge.className="toolbar-voltage-badge success"}setTaskButtonsEnabled(e){this.refs.taskButtons.forEach(t=>{const n=t.dataset.taskAction;t.disabled=e[n]===!1})}setWorkspaceButtonsEnabled(e){this.refs.workspaceButtons.forEach(t=>{const n=t.dataset.workspaceAction;t.disabled=e[n]===!1})}setDisplaySettings(e){this.refs.displayMode.value=e.mode,this.refs.gammaRange.value=e.gamma.toFixed(2),this.refs.overlayOpacityRange.value=e.overlayOpacity.toFixed(2),this.refs.displaySettingSummary.textContent=`模式=${Xh[e.mode]||e.mode} 伽马=${e.gamma.toFixed(2)} 覆盖=${e.overlayOpacity.toFixed(2)}`}setTopicLayerState(e){this.refs.topicLayerMode.value=e.mode,this.refs.pointCloudSource.value=e.pointCloudSource,this.refs.showRobotToggle.checked=!!e.showRobot,this.refs.showAxesToggle.checked=!!e.showAxes,this.refs.showPointCloudToggle.checked=!!e.showPointCloud,this.refs.showTiePointsToggle.checked=!!e.showTiePoints,this.refs.showPlanningMarkersToggle.checked=!!e.showPlanningMarkers,this.refs.pointSizeRange.value=Number(e.pointSize).toFixed(3),this.refs.pointOpacityRange.value=Number(e.pointOpacity).toFixed(2),this.refs.sceneViewMode.value=e.viewMode,this.refs.followCameraToggle.checked=!!e.followCamera,this.refs.topicLayerSummary.textContent=`模式=${Ic(e.mode)} 点云源=${zh(e.pointCloudSource)} 点大小=${Number(e.pointSize).toFixed(3)} 透明度=${Number(e.pointOpacity).toFixed(2)}`}renderTopicLayerStats(e){this.refs.topicLayerStats.innerHTML=`
      <div class="stats-card"><span>滤波点云</span><strong>${e.filteredWorldCoordCount}</strong></div>
      <div class="stats-card"><span>原始点云</span><strong>${e.rawWorldCoordCount}</strong></div>
      <div class="stats-card"><span>绑扎点</span><strong>${e.tiePointCount}</strong></div>
      <div class="stats-card"><span>规划点</span><strong>${e.planningPointCount}</strong></div>
      <div class="stats-card"><span>TF 帧</span><strong>${e.tfFrameCount}</strong></div>
    `}getDisplaySettings(){return{mode:this.refs.displayMode.value,gamma:Number.parseFloat(this.refs.gammaRange.value),overlayOpacity:Number.parseFloat(this.refs.overlayOpacityRange.value)}}renderPointList(e){if(!e.length){this.refs.selectedPoints.innerHTML='<li class="point-item">还没有点，直接在 IR 图上点 4 个角点。</li>';return}this.refs.selectedPoints.innerHTML=e.map((t,n)=>`
      <li class="point-item mono">${n+1}. x=${t.x}, y=${t.y}</li>
    `).join("")}setStatusChipState(e,t,n){const r=this.rootElement.querySelector(`[data-status-id="${e}"]`);r&&(r.className=`status-chip ${t}`,r.title=n||"")}setResultMessage(e){this.refs.resultMessage&&(this.refs.resultMessage.textContent=e,this.refs.resultMessage.title=e)}syncControlToggleStates(e={}){const t=hr();Object.entries({...t,...e}).forEach(([n,r])=>{this.setControlToggleState(n,r)})}setControlToggleState(e,t){const n=this.rootElement.querySelector(`[data-control-toggle="${e}"]`),r=cn(e);if(!n||!r)return;const s=!!(t!=null&&t.value);n.dataset.active=s?"true":"false",n.dataset.tone=(t==null?void 0:t.tone)||(s?r.activeTone:r.inactiveTone),n.classList.toggle("is-active",s),n.textContent=(t==null?void 0:t.label)||(s?r.activeLabel:r.inactiveLabel)}renderLogs(e){if(!e.length){this.refs.logList.innerHTML='<li class="log-item info">暂无终端日志，等待节点 stdout 输出。</li>';return}this.refs.logList.innerHTML=e.map(t=>`
      <li class="log-item ${t.level}">
        <div class="log-meta mono">${t.timestamp}</div>
        <div>${t.message}</div>
      </li>
    `).join("")}}const Nc="tie_robot_frontend_display_preferences";function Yh(){const i={mode:"auto",gamma:.85,overlayOpacity:.88};try{const e=localStorage.getItem(Nc);if(!e)return i;const t=JSON.parse(e);return{mode:["raw","auto","strong"].includes(t==null?void 0:t.mode)?t.mode:i.mode,gamma:Number.isFinite(t==null?void 0:t.gamma)?t.gamma:i.gamma,overlayOpacity:Number.isFinite(t==null?void 0:t.overlayOpacity)?t.overlayOpacity:i.overlayOpacity}}catch{return i}}function jh(i){try{localStorage.setItem(Nc,JSON.stringify(i))}catch{}}/**
 * @license
 * Copyright 2010-2024 Three.js Authors
 * SPDX-License-Identifier: MIT
 */const Ta="165",zn={ROTATE:0,DOLLY:1,PAN:2},Vn={ROTATE:0,PAN:1,DOLLY_PAN:2,DOLLY_ROTATE:3},$h=0,Do=1,Kh=2,Oc=1,Zh=2,Zt=3,mn=0,yt=1,Nt=2,fn=0,oi=1,Io=2,No=3,Oo=4,Jh=5,Rn=100,Qh=101,ed=102,td=103,nd=104,id=200,rd=201,sd=202,ad=203,ha=204,da=205,od=206,ld=207,cd=208,ud=209,hd=210,dd=211,fd=212,pd=213,md=214,gd=0,_d=1,vd=2,pr=3,xd=4,yd=5,Sd=6,Md=7,Fc=0,bd=1,Ed=2,pn=0,Td=1,wd=2,Ad=3,Cd=4,Rd=5,Pd=6,Ld=7,Bc=300,ui=301,hi=302,fa=303,pa=304,Er=306,ma=1e3,Ln=1001,ga=1002,Pt=1003,Ud=1004,Fi=1005,Ot=1006,Ts=1007,Un=1008,gn=1009,Dd=1010,Id=1011,mr=1012,kc=1013,di=1014,hn=1015,Tr=1016,zc=1017,Vc=1018,fi=1020,Nd=35902,Od=1021,Fd=1022,Vt=1023,Bd=1024,kd=1025,li=1026,pi=1027,zd=1028,Gc=1029,Vd=1030,Hc=1031,Wc=1033,ws=33776,As=33777,Cs=33778,Rs=33779,Fo=35840,Bo=35841,ko=35842,zo=35843,Vo=36196,Go=37492,Ho=37496,Wo=37808,Xo=37809,qo=37810,Yo=37811,jo=37812,$o=37813,Ko=37814,Zo=37815,Jo=37816,Qo=37817,el=37818,tl=37819,nl=37820,il=37821,Ps=36492,rl=36494,sl=36495,Gd=36283,al=36284,ol=36285,ll=36286,Hd=3200,Wd=3201,Xc=0,Xd=1,un="",Bt="srgb",vn="srgb-linear",wa="display-p3",wr="display-p3-linear",gr="linear",Ze="srgb",_r="rec709",vr="p3",Gn=7680,cl=519,qd=512,Yd=513,jd=514,qc=515,$d=516,Kd=517,Zd=518,Jd=519,ul=35044,hl="300 es",Jt=2e3,xr=2001;class Bn{addEventListener(e,t){this._listeners===void 0&&(this._listeners={});const n=this._listeners;n[e]===void 0&&(n[e]=[]),n[e].indexOf(t)===-1&&n[e].push(t)}hasEventListener(e,t){if(this._listeners===void 0)return!1;const n=this._listeners;return n[e]!==void 0&&n[e].indexOf(t)!==-1}removeEventListener(e,t){if(this._listeners===void 0)return;const r=this._listeners[e];if(r!==void 0){const s=r.indexOf(t);s!==-1&&r.splice(s,1)}}dispatchEvent(e){if(this._listeners===void 0)return;const n=this._listeners[e.type];if(n!==void 0){e.target=this;const r=n.slice(0);for(let s=0,o=r.length;s<o;s++)r[s].call(this,e);e.target=null}}}const pt=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"],dr=Math.PI/180,_a=180/Math.PI;function Ui(){const i=Math.random()*4294967295|0,e=Math.random()*4294967295|0,t=Math.random()*4294967295|0,n=Math.random()*4294967295|0;return(pt[i&255]+pt[i>>8&255]+pt[i>>16&255]+pt[i>>24&255]+"-"+pt[e&255]+pt[e>>8&255]+"-"+pt[e>>16&15|64]+pt[e>>24&255]+"-"+pt[t&63|128]+pt[t>>8&255]+"-"+pt[t>>16&255]+pt[t>>24&255]+pt[n&255]+pt[n>>8&255]+pt[n>>16&255]+pt[n>>24&255]).toLowerCase()}function vt(i,e,t){return Math.max(e,Math.min(t,i))}function Qd(i,e){return(i%e+e)%e}function Ls(i,e,t){return(1-t)*i+t*e}function yi(i,e){switch(e.constructor){case Float32Array:return i;case Uint32Array:return i/4294967295;case Uint16Array:return i/65535;case Uint8Array:return i/255;case Int32Array:return Math.max(i/2147483647,-1);case Int16Array:return Math.max(i/32767,-1);case Int8Array:return Math.max(i/127,-1);default:throw new Error("Invalid component type.")}}function xt(i,e){switch(e.constructor){case Float32Array:return i;case Uint32Array:return Math.round(i*4294967295);case Uint16Array:return Math.round(i*65535);case Uint8Array:return Math.round(i*255);case Int32Array:return Math.round(i*2147483647);case Int16Array:return Math.round(i*32767);case Int8Array:return Math.round(i*127);default:throw new Error("Invalid component type.")}}const ef={DEG2RAD:dr};class De{constructor(e=0,t=0){De.prototype.isVector2=!0,this.x=e,this.y=t}get width(){return this.x}set width(e){this.x=e}get height(){return this.y}set height(e){this.y=e}set(e,t){return this.x=e,this.y=t,this}setScalar(e){return this.x=e,this.y=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y)}copy(e){return this.x=e.x,this.y=e.y,this}add(e){return this.x+=e.x,this.y+=e.y,this}addScalar(e){return this.x+=e,this.y+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this}subScalar(e){return this.x-=e,this.y-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this}multiply(e){return this.x*=e.x,this.y*=e.y,this}multiplyScalar(e){return this.x*=e,this.y*=e,this}divide(e){return this.x/=e.x,this.y/=e.y,this}divideScalar(e){return this.multiplyScalar(1/e)}applyMatrix3(e){const t=this.x,n=this.y,r=e.elements;return this.x=r[0]*t+r[3]*n+r[6],this.y=r[1]*t+r[4]*n+r[7],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this}clampLength(e,t){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(e,Math.min(t,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(e){return this.x*e.x+this.y*e.y}cross(e){return this.x*e.y-this.y*e.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const n=this.dot(e)/t;return Math.acos(vt(n,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,n=this.y-e.y;return t*t+n*n}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this}equals(e){return e.x===this.x&&e.y===this.y}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this}rotateAround(e,t){const n=Math.cos(t),r=Math.sin(t),s=this.x-e.x,o=this.y-e.y;return this.x=s*n-o*r+e.x,this.y=s*r+o*n+e.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class ze{constructor(e,t,n,r,s,o,a,l,c){ze.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],e!==void 0&&this.set(e,t,n,r,s,o,a,l,c)}set(e,t,n,r,s,o,a,l,c){const u=this.elements;return u[0]=e,u[1]=r,u[2]=a,u[3]=t,u[4]=s,u[5]=l,u[6]=n,u[7]=o,u[8]=c,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(e){const t=this.elements,n=e.elements;return t[0]=n[0],t[1]=n[1],t[2]=n[2],t[3]=n[3],t[4]=n[4],t[5]=n[5],t[6]=n[6],t[7]=n[7],t[8]=n[8],this}extractBasis(e,t,n){return e.setFromMatrix3Column(this,0),t.setFromMatrix3Column(this,1),n.setFromMatrix3Column(this,2),this}setFromMatrix4(e){const t=e.elements;return this.set(t[0],t[4],t[8],t[1],t[5],t[9],t[2],t[6],t[10]),this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const n=e.elements,r=t.elements,s=this.elements,o=n[0],a=n[3],l=n[6],c=n[1],u=n[4],h=n[7],d=n[2],f=n[5],g=n[8],x=r[0],m=r[3],p=r[6],T=r[1],S=r[4],A=r[7],B=r[2],D=r[5],P=r[8];return s[0]=o*x+a*T+l*B,s[3]=o*m+a*S+l*D,s[6]=o*p+a*A+l*P,s[1]=c*x+u*T+h*B,s[4]=c*m+u*S+h*D,s[7]=c*p+u*A+h*P,s[2]=d*x+f*T+g*B,s[5]=d*m+f*S+g*D,s[8]=d*p+f*A+g*P,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[3]*=e,t[6]*=e,t[1]*=e,t[4]*=e,t[7]*=e,t[2]*=e,t[5]*=e,t[8]*=e,this}determinant(){const e=this.elements,t=e[0],n=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8];return t*o*u-t*a*c-n*s*u+n*a*l+r*s*c-r*o*l}invert(){const e=this.elements,t=e[0],n=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8],h=u*o-a*c,d=a*l-u*s,f=c*s-o*l,g=t*h+n*d+r*f;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);const x=1/g;return e[0]=h*x,e[1]=(r*c-u*n)*x,e[2]=(a*n-r*o)*x,e[3]=d*x,e[4]=(u*t-r*l)*x,e[5]=(r*s-a*t)*x,e[6]=f*x,e[7]=(n*l-c*t)*x,e[8]=(o*t-n*s)*x,this}transpose(){let e;const t=this.elements;return e=t[1],t[1]=t[3],t[3]=e,e=t[2],t[2]=t[6],t[6]=e,e=t[5],t[5]=t[7],t[7]=e,this}getNormalMatrix(e){return this.setFromMatrix4(e).invert().transpose()}transposeIntoArray(e){const t=this.elements;return e[0]=t[0],e[1]=t[3],e[2]=t[6],e[3]=t[1],e[4]=t[4],e[5]=t[7],e[6]=t[2],e[7]=t[5],e[8]=t[8],this}setUvTransform(e,t,n,r,s,o,a){const l=Math.cos(s),c=Math.sin(s);return this.set(n*l,n*c,-n*(l*o+c*a)+o+e,-r*c,r*l,-r*(-c*o+l*a)+a+t,0,0,1),this}scale(e,t){return this.premultiply(Us.makeScale(e,t)),this}rotate(e){return this.premultiply(Us.makeRotation(-e)),this}translate(e,t){return this.premultiply(Us.makeTranslation(e,t)),this}makeTranslation(e,t){return e.isVector2?this.set(1,0,e.x,0,1,e.y,0,0,1):this.set(1,0,e,0,1,t,0,0,1),this}makeRotation(e){const t=Math.cos(e),n=Math.sin(e);return this.set(t,-n,0,n,t,0,0,0,1),this}makeScale(e,t){return this.set(e,0,0,0,t,0,0,0,1),this}equals(e){const t=this.elements,n=e.elements;for(let r=0;r<9;r++)if(t[r]!==n[r])return!1;return!0}fromArray(e,t=0){for(let n=0;n<9;n++)this.elements[n]=e[n+t];return this}toArray(e=[],t=0){const n=this.elements;return e[t]=n[0],e[t+1]=n[1],e[t+2]=n[2],e[t+3]=n[3],e[t+4]=n[4],e[t+5]=n[5],e[t+6]=n[6],e[t+7]=n[7],e[t+8]=n[8],e}clone(){return new this.constructor().fromArray(this.elements)}}const Us=new ze;function Yc(i){for(let e=i.length-1;e>=0;--e)if(i[e]>=65535)return!0;return!1}function yr(i){return document.createElementNS("http://www.w3.org/1999/xhtml",i)}function tf(){const i=yr("canvas");return i.style.display="block",i}const dl={};function jc(i){i in dl||(dl[i]=!0,console.warn(i))}function nf(i,e,t){return new Promise(function(n,r){function s(){switch(i.clientWaitSync(e,i.SYNC_FLUSH_COMMANDS_BIT,0)){case i.WAIT_FAILED:r();break;case i.TIMEOUT_EXPIRED:setTimeout(s,t);break;default:n()}}setTimeout(s,t)})}const fl=new ze().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),pl=new ze().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),Bi={[vn]:{transfer:gr,primaries:_r,toReference:i=>i,fromReference:i=>i},[Bt]:{transfer:Ze,primaries:_r,toReference:i=>i.convertSRGBToLinear(),fromReference:i=>i.convertLinearToSRGB()},[wr]:{transfer:gr,primaries:vr,toReference:i=>i.applyMatrix3(pl),fromReference:i=>i.applyMatrix3(fl)},[wa]:{transfer:Ze,primaries:vr,toReference:i=>i.convertSRGBToLinear().applyMatrix3(pl),fromReference:i=>i.applyMatrix3(fl).convertLinearToSRGB()}},rf=new Set([vn,wr]),je={enabled:!0,_workingColorSpace:vn,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(i){if(!rf.has(i))throw new Error(`Unsupported working color space, "${i}".`);this._workingColorSpace=i},convert:function(i,e,t){if(this.enabled===!1||e===t||!e||!t)return i;const n=Bi[e].toReference,r=Bi[t].fromReference;return r(n(i))},fromWorkingColorSpace:function(i,e){return this.convert(i,this._workingColorSpace,e)},toWorkingColorSpace:function(i,e){return this.convert(i,e,this._workingColorSpace)},getPrimaries:function(i){return Bi[i].primaries},getTransfer:function(i){return i===un?gr:Bi[i].transfer}};function ci(i){return i<.04045?i*.0773993808:Math.pow(i*.9478672986+.0521327014,2.4)}function Ds(i){return i<.0031308?i*12.92:1.055*Math.pow(i,.41666)-.055}let Hn;class sf{static getDataURL(e){if(/^data:/i.test(e.src)||typeof HTMLCanvasElement>"u")return e.src;let t;if(e instanceof HTMLCanvasElement)t=e;else{Hn===void 0&&(Hn=yr("canvas")),Hn.width=e.width,Hn.height=e.height;const n=Hn.getContext("2d");e instanceof ImageData?n.putImageData(e,0,0):n.drawImage(e,0,0,e.width,e.height),t=Hn}return t.width>2048||t.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",e),t.toDataURL("image/jpeg",.6)):t.toDataURL("image/png")}static sRGBToLinear(e){if(typeof HTMLImageElement<"u"&&e instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&e instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&e instanceof ImageBitmap){const t=yr("canvas");t.width=e.width,t.height=e.height;const n=t.getContext("2d");n.drawImage(e,0,0,e.width,e.height);const r=n.getImageData(0,0,e.width,e.height),s=r.data;for(let o=0;o<s.length;o++)s[o]=ci(s[o]/255)*255;return n.putImageData(r,0,0),t}else if(e.data){const t=e.data.slice(0);for(let n=0;n<t.length;n++)t instanceof Uint8Array||t instanceof Uint8ClampedArray?t[n]=Math.floor(ci(t[n]/255)*255):t[n]=ci(t[n]);return{data:t,width:e.width,height:e.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),e}}let af=0;class $c{constructor(e=null){this.isSource=!0,Object.defineProperty(this,"id",{value:af++}),this.uuid=Ui(),this.data=e,this.dataReady=!0,this.version=0}set needsUpdate(e){e===!0&&this.version++}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.images[this.uuid]!==void 0)return e.images[this.uuid];const n={uuid:this.uuid,url:""},r=this.data;if(r!==null){let s;if(Array.isArray(r)){s=[];for(let o=0,a=r.length;o<a;o++)r[o].isDataTexture?s.push(Is(r[o].image)):s.push(Is(r[o]))}else s=Is(r);n.url=s}return t||(e.images[this.uuid]=n),n}}function Is(i){return typeof HTMLImageElement<"u"&&i instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&i instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&i instanceof ImageBitmap?sf.getDataURL(i):i.data?{data:Array.from(i.data),width:i.width,height:i.height,type:i.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let of=0;class St extends Bn{constructor(e=St.DEFAULT_IMAGE,t=St.DEFAULT_MAPPING,n=Ln,r=Ln,s=Ot,o=Un,a=Vt,l=gn,c=St.DEFAULT_ANISOTROPY,u=un){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:of++}),this.uuid=Ui(),this.name="",this.source=new $c(e),this.mipmaps=[],this.mapping=t,this.channel=0,this.wrapS=n,this.wrapT=r,this.magFilter=s,this.minFilter=o,this.anisotropy=c,this.format=a,this.internalFormat=null,this.type=l,this.offset=new De(0,0),this.repeat=new De(1,1),this.center=new De(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new ze,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=u,this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.pmremVersion=0}get image(){return this.source.data}set image(e=null){this.source.data=e}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(e){return this.name=e.name,this.source=e.source,this.mipmaps=e.mipmaps.slice(0),this.mapping=e.mapping,this.channel=e.channel,this.wrapS=e.wrapS,this.wrapT=e.wrapT,this.magFilter=e.magFilter,this.minFilter=e.minFilter,this.anisotropy=e.anisotropy,this.format=e.format,this.internalFormat=e.internalFormat,this.type=e.type,this.offset.copy(e.offset),this.repeat.copy(e.repeat),this.center.copy(e.center),this.rotation=e.rotation,this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrix.copy(e.matrix),this.generateMipmaps=e.generateMipmaps,this.premultiplyAlpha=e.premultiplyAlpha,this.flipY=e.flipY,this.unpackAlignment=e.unpackAlignment,this.colorSpace=e.colorSpace,this.userData=JSON.parse(JSON.stringify(e.userData)),this.needsUpdate=!0,this}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.textures[this.uuid]!==void 0)return e.textures[this.uuid];const n={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(e).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(n.userData=this.userData),t||(e.textures[this.uuid]=n),n}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(e){if(this.mapping!==Bc)return e;if(e.applyMatrix3(this.matrix),e.x<0||e.x>1)switch(this.wrapS){case ma:e.x=e.x-Math.floor(e.x);break;case Ln:e.x=e.x<0?0:1;break;case ga:Math.abs(Math.floor(e.x)%2)===1?e.x=Math.ceil(e.x)-e.x:e.x=e.x-Math.floor(e.x);break}if(e.y<0||e.y>1)switch(this.wrapT){case ma:e.y=e.y-Math.floor(e.y);break;case Ln:e.y=e.y<0?0:1;break;case ga:Math.abs(Math.floor(e.y)%2)===1?e.y=Math.ceil(e.y)-e.y:e.y=e.y-Math.floor(e.y);break}return this.flipY&&(e.y=1-e.y),e}set needsUpdate(e){e===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(e){e===!0&&this.pmremVersion++}}St.DEFAULT_IMAGE=null;St.DEFAULT_MAPPING=Bc;St.DEFAULT_ANISOTROPY=1;class ht{constructor(e=0,t=0,n=0,r=1){ht.prototype.isVector4=!0,this.x=e,this.y=t,this.z=n,this.w=r}get width(){return this.z}set width(e){this.z=e}get height(){return this.w}set height(e){this.w=e}set(e,t,n,r){return this.x=e,this.y=t,this.z=n,this.w=r,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this.w=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setW(e){return this.w=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;case 3:this.w=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this.w=e.w!==void 0?e.w:1,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this.w+=e.w,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this.w+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this.w=e.w+t.w,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this.w+=e.w*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this.w-=e.w,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this.w-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this.w=e.w-t.w,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this.w*=e.w,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this.w*=e,this}applyMatrix4(e){const t=this.x,n=this.y,r=this.z,s=this.w,o=e.elements;return this.x=o[0]*t+o[4]*n+o[8]*r+o[12]*s,this.y=o[1]*t+o[5]*n+o[9]*r+o[13]*s,this.z=o[2]*t+o[6]*n+o[10]*r+o[14]*s,this.w=o[3]*t+o[7]*n+o[11]*r+o[15]*s,this}divideScalar(e){return this.multiplyScalar(1/e)}setAxisAngleFromQuaternion(e){this.w=2*Math.acos(e.w);const t=Math.sqrt(1-e.w*e.w);return t<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=e.x/t,this.y=e.y/t,this.z=e.z/t),this}setAxisAngleFromRotationMatrix(e){let t,n,r,s;const l=e.elements,c=l[0],u=l[4],h=l[8],d=l[1],f=l[5],g=l[9],x=l[2],m=l[6],p=l[10];if(Math.abs(u-d)<.01&&Math.abs(h-x)<.01&&Math.abs(g-m)<.01){if(Math.abs(u+d)<.1&&Math.abs(h+x)<.1&&Math.abs(g+m)<.1&&Math.abs(c+f+p-3)<.1)return this.set(1,0,0,0),this;t=Math.PI;const S=(c+1)/2,A=(f+1)/2,B=(p+1)/2,D=(u+d)/4,P=(h+x)/4,X=(g+m)/4;return S>A&&S>B?S<.01?(n=0,r=.707106781,s=.707106781):(n=Math.sqrt(S),r=D/n,s=P/n):A>B?A<.01?(n=.707106781,r=0,s=.707106781):(r=Math.sqrt(A),n=D/r,s=X/r):B<.01?(n=.707106781,r=.707106781,s=0):(s=Math.sqrt(B),n=P/s,r=X/s),this.set(n,r,s,t),this}let T=Math.sqrt((m-g)*(m-g)+(h-x)*(h-x)+(d-u)*(d-u));return Math.abs(T)<.001&&(T=1),this.x=(m-g)/T,this.y=(h-x)/T,this.z=(d-u)/T,this.w=Math.acos((c+f+p-1)/2),this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this.w=Math.min(this.w,e.w),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this.w=Math.max(this.w,e.w),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this.w=Math.max(e.w,Math.min(t.w,this.w)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this.w=Math.max(e,Math.min(t,this.w)),this}clampLength(e,t){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(e,Math.min(t,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z+this.w*e.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this.w+=(e.w-this.w)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this.z=e.z+(t.z-e.z)*n,this.w=e.w+(t.w-e.w)*n,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z&&e.w===this.w}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this.w=e[t+3],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e[t+3]=this.w,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this.w=e.getW(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class lf extends Bn{constructor(e=1,t=1,n={}){super(),this.isRenderTarget=!0,this.width=e,this.height=t,this.depth=1,this.scissor=new ht(0,0,e,t),this.scissorTest=!1,this.viewport=new ht(0,0,e,t);const r={width:e,height:t,depth:1};n=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:Ot,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1},n);const s=new St(r,n.mapping,n.wrapS,n.wrapT,n.magFilter,n.minFilter,n.format,n.type,n.anisotropy,n.colorSpace);s.flipY=!1,s.generateMipmaps=n.generateMipmaps,s.internalFormat=n.internalFormat,this.textures=[];const o=n.count;for(let a=0;a<o;a++)this.textures[a]=s.clone(),this.textures[a].isRenderTargetTexture=!0;this.depthBuffer=n.depthBuffer,this.stencilBuffer=n.stencilBuffer,this.resolveDepthBuffer=n.resolveDepthBuffer,this.resolveStencilBuffer=n.resolveStencilBuffer,this.depthTexture=n.depthTexture,this.samples=n.samples}get texture(){return this.textures[0]}set texture(e){this.textures[0]=e}setSize(e,t,n=1){if(this.width!==e||this.height!==t||this.depth!==n){this.width=e,this.height=t,this.depth=n;for(let r=0,s=this.textures.length;r<s;r++)this.textures[r].image.width=e,this.textures[r].image.height=t,this.textures[r].image.depth=n;this.dispose()}this.viewport.set(0,0,e,t),this.scissor.set(0,0,e,t)}clone(){return new this.constructor().copy(this)}copy(e){this.width=e.width,this.height=e.height,this.depth=e.depth,this.scissor.copy(e.scissor),this.scissorTest=e.scissorTest,this.viewport.copy(e.viewport),this.textures.length=0;for(let n=0,r=e.textures.length;n<r;n++)this.textures[n]=e.textures[n].clone(),this.textures[n].isRenderTargetTexture=!0;const t=Object.assign({},e.texture.image);return this.texture.source=new $c(t),this.depthBuffer=e.depthBuffer,this.stencilBuffer=e.stencilBuffer,this.resolveDepthBuffer=e.resolveDepthBuffer,this.resolveStencilBuffer=e.resolveStencilBuffer,e.depthTexture!==null&&(this.depthTexture=e.depthTexture.clone()),this.samples=e.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class Dn extends lf{constructor(e=1,t=1,n={}){super(e,t,n),this.isWebGLRenderTarget=!0}}class Kc extends St{constructor(e=null,t=1,n=1,r=1){super(null),this.isDataArrayTexture=!0,this.image={data:e,width:t,height:n,depth:r},this.magFilter=Pt,this.minFilter=Pt,this.wrapR=Ln,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(e){this.layerUpdates.add(e)}clearLayerUpdates(){this.layerUpdates.clear()}}class cf extends St{constructor(e=null,t=1,n=1,r=1){super(null),this.isData3DTexture=!0,this.image={data:e,width:t,height:n,depth:r},this.magFilter=Pt,this.minFilter=Pt,this.wrapR=Ln,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class Ht{constructor(e=0,t=0,n=0,r=1){this.isQuaternion=!0,this._x=e,this._y=t,this._z=n,this._w=r}static slerpFlat(e,t,n,r,s,o,a){let l=n[r+0],c=n[r+1],u=n[r+2],h=n[r+3];const d=s[o+0],f=s[o+1],g=s[o+2],x=s[o+3];if(a===0){e[t+0]=l,e[t+1]=c,e[t+2]=u,e[t+3]=h;return}if(a===1){e[t+0]=d,e[t+1]=f,e[t+2]=g,e[t+3]=x;return}if(h!==x||l!==d||c!==f||u!==g){let m=1-a;const p=l*d+c*f+u*g+h*x,T=p>=0?1:-1,S=1-p*p;if(S>Number.EPSILON){const B=Math.sqrt(S),D=Math.atan2(B,p*T);m=Math.sin(m*D)/B,a=Math.sin(a*D)/B}const A=a*T;if(l=l*m+d*A,c=c*m+f*A,u=u*m+g*A,h=h*m+x*A,m===1-a){const B=1/Math.sqrt(l*l+c*c+u*u+h*h);l*=B,c*=B,u*=B,h*=B}}e[t]=l,e[t+1]=c,e[t+2]=u,e[t+3]=h}static multiplyQuaternionsFlat(e,t,n,r,s,o){const a=n[r],l=n[r+1],c=n[r+2],u=n[r+3],h=s[o],d=s[o+1],f=s[o+2],g=s[o+3];return e[t]=a*g+u*h+l*f-c*d,e[t+1]=l*g+u*d+c*h-a*f,e[t+2]=c*g+u*f+a*d-l*h,e[t+3]=u*g-a*h-l*d-c*f,e}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get w(){return this._w}set w(e){this._w=e,this._onChangeCallback()}set(e,t,n,r){return this._x=e,this._y=t,this._z=n,this._w=r,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(e){return this._x=e.x,this._y=e.y,this._z=e.z,this._w=e.w,this._onChangeCallback(),this}setFromEuler(e,t=!0){const n=e._x,r=e._y,s=e._z,o=e._order,a=Math.cos,l=Math.sin,c=a(n/2),u=a(r/2),h=a(s/2),d=l(n/2),f=l(r/2),g=l(s/2);switch(o){case"XYZ":this._x=d*u*h+c*f*g,this._y=c*f*h-d*u*g,this._z=c*u*g+d*f*h,this._w=c*u*h-d*f*g;break;case"YXZ":this._x=d*u*h+c*f*g,this._y=c*f*h-d*u*g,this._z=c*u*g-d*f*h,this._w=c*u*h+d*f*g;break;case"ZXY":this._x=d*u*h-c*f*g,this._y=c*f*h+d*u*g,this._z=c*u*g+d*f*h,this._w=c*u*h-d*f*g;break;case"ZYX":this._x=d*u*h-c*f*g,this._y=c*f*h+d*u*g,this._z=c*u*g-d*f*h,this._w=c*u*h+d*f*g;break;case"YZX":this._x=d*u*h+c*f*g,this._y=c*f*h+d*u*g,this._z=c*u*g-d*f*h,this._w=c*u*h-d*f*g;break;case"XZY":this._x=d*u*h-c*f*g,this._y=c*f*h-d*u*g,this._z=c*u*g+d*f*h,this._w=c*u*h+d*f*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+o)}return t===!0&&this._onChangeCallback(),this}setFromAxisAngle(e,t){const n=t/2,r=Math.sin(n);return this._x=e.x*r,this._y=e.y*r,this._z=e.z*r,this._w=Math.cos(n),this._onChangeCallback(),this}setFromRotationMatrix(e){const t=e.elements,n=t[0],r=t[4],s=t[8],o=t[1],a=t[5],l=t[9],c=t[2],u=t[6],h=t[10],d=n+a+h;if(d>0){const f=.5/Math.sqrt(d+1);this._w=.25/f,this._x=(u-l)*f,this._y=(s-c)*f,this._z=(o-r)*f}else if(n>a&&n>h){const f=2*Math.sqrt(1+n-a-h);this._w=(u-l)/f,this._x=.25*f,this._y=(r+o)/f,this._z=(s+c)/f}else if(a>h){const f=2*Math.sqrt(1+a-n-h);this._w=(s-c)/f,this._x=(r+o)/f,this._y=.25*f,this._z=(l+u)/f}else{const f=2*Math.sqrt(1+h-n-a);this._w=(o-r)/f,this._x=(s+c)/f,this._y=(l+u)/f,this._z=.25*f}return this._onChangeCallback(),this}setFromUnitVectors(e,t){let n=e.dot(t)+1;return n<Number.EPSILON?(n=0,Math.abs(e.x)>Math.abs(e.z)?(this._x=-e.y,this._y=e.x,this._z=0,this._w=n):(this._x=0,this._y=-e.z,this._z=e.y,this._w=n)):(this._x=e.y*t.z-e.z*t.y,this._y=e.z*t.x-e.x*t.z,this._z=e.x*t.y-e.y*t.x,this._w=n),this.normalize()}angleTo(e){return 2*Math.acos(Math.abs(vt(this.dot(e),-1,1)))}rotateTowards(e,t){const n=this.angleTo(e);if(n===0)return this;const r=Math.min(1,t/n);return this.slerp(e,r),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(e){return this._x*e._x+this._y*e._y+this._z*e._z+this._w*e._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let e=this.length();return e===0?(this._x=0,this._y=0,this._z=0,this._w=1):(e=1/e,this._x=this._x*e,this._y=this._y*e,this._z=this._z*e,this._w=this._w*e),this._onChangeCallback(),this}multiply(e){return this.multiplyQuaternions(this,e)}premultiply(e){return this.multiplyQuaternions(e,this)}multiplyQuaternions(e,t){const n=e._x,r=e._y,s=e._z,o=e._w,a=t._x,l=t._y,c=t._z,u=t._w;return this._x=n*u+o*a+r*c-s*l,this._y=r*u+o*l+s*a-n*c,this._z=s*u+o*c+n*l-r*a,this._w=o*u-n*a-r*l-s*c,this._onChangeCallback(),this}slerp(e,t){if(t===0)return this;if(t===1)return this.copy(e);const n=this._x,r=this._y,s=this._z,o=this._w;let a=o*e._w+n*e._x+r*e._y+s*e._z;if(a<0?(this._w=-e._w,this._x=-e._x,this._y=-e._y,this._z=-e._z,a=-a):this.copy(e),a>=1)return this._w=o,this._x=n,this._y=r,this._z=s,this;const l=1-a*a;if(l<=Number.EPSILON){const f=1-t;return this._w=f*o+t*this._w,this._x=f*n+t*this._x,this._y=f*r+t*this._y,this._z=f*s+t*this._z,this.normalize(),this}const c=Math.sqrt(l),u=Math.atan2(c,a),h=Math.sin((1-t)*u)/c,d=Math.sin(t*u)/c;return this._w=o*h+this._w*d,this._x=n*h+this._x*d,this._y=r*h+this._y*d,this._z=s*h+this._z*d,this._onChangeCallback(),this}slerpQuaternions(e,t,n){return this.copy(e).slerp(t,n)}random(){const e=2*Math.PI*Math.random(),t=2*Math.PI*Math.random(),n=Math.random(),r=Math.sqrt(1-n),s=Math.sqrt(n);return this.set(r*Math.sin(e),r*Math.cos(e),s*Math.sin(t),s*Math.cos(t))}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._w===this._w}fromArray(e,t=0){return this._x=e[t],this._y=e[t+1],this._z=e[t+2],this._w=e[t+3],this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._w,e}fromBufferAttribute(e,t){return this._x=e.getX(t),this._y=e.getY(t),this._z=e.getZ(t),this._w=e.getW(t),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class z{constructor(e=0,t=0,n=0){z.prototype.isVector3=!0,this.x=e,this.y=t,this.z=n}set(e,t,n){return n===void 0&&(n=this.z),this.x=e,this.y=t,this.z=n,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this}multiplyVectors(e,t){return this.x=e.x*t.x,this.y=e.y*t.y,this.z=e.z*t.z,this}applyEuler(e){return this.applyQuaternion(ml.setFromEuler(e))}applyAxisAngle(e,t){return this.applyQuaternion(ml.setFromAxisAngle(e,t))}applyMatrix3(e){const t=this.x,n=this.y,r=this.z,s=e.elements;return this.x=s[0]*t+s[3]*n+s[6]*r,this.y=s[1]*t+s[4]*n+s[7]*r,this.z=s[2]*t+s[5]*n+s[8]*r,this}applyNormalMatrix(e){return this.applyMatrix3(e).normalize()}applyMatrix4(e){const t=this.x,n=this.y,r=this.z,s=e.elements,o=1/(s[3]*t+s[7]*n+s[11]*r+s[15]);return this.x=(s[0]*t+s[4]*n+s[8]*r+s[12])*o,this.y=(s[1]*t+s[5]*n+s[9]*r+s[13])*o,this.z=(s[2]*t+s[6]*n+s[10]*r+s[14])*o,this}applyQuaternion(e){const t=this.x,n=this.y,r=this.z,s=e.x,o=e.y,a=e.z,l=e.w,c=2*(o*r-a*n),u=2*(a*t-s*r),h=2*(s*n-o*t);return this.x=t+l*c+o*h-a*u,this.y=n+l*u+a*c-s*h,this.z=r+l*h+s*u-o*c,this}project(e){return this.applyMatrix4(e.matrixWorldInverse).applyMatrix4(e.projectionMatrix)}unproject(e){return this.applyMatrix4(e.projectionMatrixInverse).applyMatrix4(e.matrixWorld)}transformDirection(e){const t=this.x,n=this.y,r=this.z,s=e.elements;return this.x=s[0]*t+s[4]*n+s[8]*r,this.y=s[1]*t+s[5]*n+s[9]*r,this.z=s[2]*t+s[6]*n+s[10]*r,this.normalize()}divide(e){return this.x/=e.x,this.y/=e.y,this.z/=e.z,this}divideScalar(e){return this.multiplyScalar(1/e)}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this}clampLength(e,t){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(e,Math.min(t,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this.z=e.z+(t.z-e.z)*n,this}cross(e){return this.crossVectors(this,e)}crossVectors(e,t){const n=e.x,r=e.y,s=e.z,o=t.x,a=t.y,l=t.z;return this.x=r*l-s*a,this.y=s*o-n*l,this.z=n*a-r*o,this}projectOnVector(e){const t=e.lengthSq();if(t===0)return this.set(0,0,0);const n=e.dot(this)/t;return this.copy(e).multiplyScalar(n)}projectOnPlane(e){return Ns.copy(this).projectOnVector(e),this.sub(Ns)}reflect(e){return this.sub(Ns.copy(e).multiplyScalar(2*this.dot(e)))}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const n=this.dot(e)/t;return Math.acos(vt(n,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,n=this.y-e.y,r=this.z-e.z;return t*t+n*n+r*r}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)+Math.abs(this.z-e.z)}setFromSpherical(e){return this.setFromSphericalCoords(e.radius,e.phi,e.theta)}setFromSphericalCoords(e,t,n){const r=Math.sin(t)*e;return this.x=r*Math.sin(n),this.y=Math.cos(t)*e,this.z=r*Math.cos(n),this}setFromCylindrical(e){return this.setFromCylindricalCoords(e.radius,e.theta,e.y)}setFromCylindricalCoords(e,t,n){return this.x=e*Math.sin(t),this.y=n,this.z=e*Math.cos(t),this}setFromMatrixPosition(e){const t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this}setFromMatrixScale(e){const t=this.setFromMatrixColumn(e,0).length(),n=this.setFromMatrixColumn(e,1).length(),r=this.setFromMatrixColumn(e,2).length();return this.x=t,this.y=n,this.z=r,this}setFromMatrixColumn(e,t){return this.fromArray(e.elements,t*4)}setFromMatrix3Column(e,t){return this.fromArray(e.elements,t*3)}setFromEuler(e){return this.x=e._x,this.y=e._y,this.z=e._z,this}setFromColor(e){return this.x=e.r,this.y=e.g,this.z=e.b,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const e=Math.random()*Math.PI*2,t=Math.random()*2-1,n=Math.sqrt(1-t*t);return this.x=n*Math.cos(e),this.y=t,this.z=n*Math.sin(e),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const Ns=new z,ml=new Ht;class Di{constructor(e=new z(1/0,1/0,1/0),t=new z(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=e,this.max=t}set(e,t){return this.min.copy(e),this.max.copy(t),this}setFromArray(e){this.makeEmpty();for(let t=0,n=e.length;t<n;t+=3)this.expandByPoint(Ut.fromArray(e,t));return this}setFromBufferAttribute(e){this.makeEmpty();for(let t=0,n=e.count;t<n;t++)this.expandByPoint(Ut.fromBufferAttribute(e,t));return this}setFromPoints(e){this.makeEmpty();for(let t=0,n=e.length;t<n;t++)this.expandByPoint(e[t]);return this}setFromCenterAndSize(e,t){const n=Ut.copy(t).multiplyScalar(.5);return this.min.copy(e).sub(n),this.max.copy(e).add(n),this}setFromObject(e,t=!1){return this.makeEmpty(),this.expandByObject(e,t)}clone(){return new this.constructor().copy(this)}copy(e){return this.min.copy(e.min),this.max.copy(e.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(e){return this.isEmpty()?e.set(0,0,0):e.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(e){return this.isEmpty()?e.set(0,0,0):e.subVectors(this.max,this.min)}expandByPoint(e){return this.min.min(e),this.max.max(e),this}expandByVector(e){return this.min.sub(e),this.max.add(e),this}expandByScalar(e){return this.min.addScalar(-e),this.max.addScalar(e),this}expandByObject(e,t=!1){e.updateWorldMatrix(!1,!1);const n=e.geometry;if(n!==void 0){const s=n.getAttribute("position");if(t===!0&&s!==void 0&&e.isInstancedMesh!==!0)for(let o=0,a=s.count;o<a;o++)e.isMesh===!0?e.getVertexPosition(o,Ut):Ut.fromBufferAttribute(s,o),Ut.applyMatrix4(e.matrixWorld),this.expandByPoint(Ut);else e.boundingBox!==void 0?(e.boundingBox===null&&e.computeBoundingBox(),ki.copy(e.boundingBox)):(n.boundingBox===null&&n.computeBoundingBox(),ki.copy(n.boundingBox)),ki.applyMatrix4(e.matrixWorld),this.union(ki)}const r=e.children;for(let s=0,o=r.length;s<o;s++)this.expandByObject(r[s],t);return this}containsPoint(e){return!(e.x<this.min.x||e.x>this.max.x||e.y<this.min.y||e.y>this.max.y||e.z<this.min.z||e.z>this.max.z)}containsBox(e){return this.min.x<=e.min.x&&e.max.x<=this.max.x&&this.min.y<=e.min.y&&e.max.y<=this.max.y&&this.min.z<=e.min.z&&e.max.z<=this.max.z}getParameter(e,t){return t.set((e.x-this.min.x)/(this.max.x-this.min.x),(e.y-this.min.y)/(this.max.y-this.min.y),(e.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(e){return!(e.max.x<this.min.x||e.min.x>this.max.x||e.max.y<this.min.y||e.min.y>this.max.y||e.max.z<this.min.z||e.min.z>this.max.z)}intersectsSphere(e){return this.clampPoint(e.center,Ut),Ut.distanceToSquared(e.center)<=e.radius*e.radius}intersectsPlane(e){let t,n;return e.normal.x>0?(t=e.normal.x*this.min.x,n=e.normal.x*this.max.x):(t=e.normal.x*this.max.x,n=e.normal.x*this.min.x),e.normal.y>0?(t+=e.normal.y*this.min.y,n+=e.normal.y*this.max.y):(t+=e.normal.y*this.max.y,n+=e.normal.y*this.min.y),e.normal.z>0?(t+=e.normal.z*this.min.z,n+=e.normal.z*this.max.z):(t+=e.normal.z*this.max.z,n+=e.normal.z*this.min.z),t<=-e.constant&&n>=-e.constant}intersectsTriangle(e){if(this.isEmpty())return!1;this.getCenter(Si),zi.subVectors(this.max,Si),Wn.subVectors(e.a,Si),Xn.subVectors(e.b,Si),qn.subVectors(e.c,Si),tn.subVectors(Xn,Wn),nn.subVectors(qn,Xn),Mn.subVectors(Wn,qn);let t=[0,-tn.z,tn.y,0,-nn.z,nn.y,0,-Mn.z,Mn.y,tn.z,0,-tn.x,nn.z,0,-nn.x,Mn.z,0,-Mn.x,-tn.y,tn.x,0,-nn.y,nn.x,0,-Mn.y,Mn.x,0];return!Os(t,Wn,Xn,qn,zi)||(t=[1,0,0,0,1,0,0,0,1],!Os(t,Wn,Xn,qn,zi))?!1:(Vi.crossVectors(tn,nn),t=[Vi.x,Vi.y,Vi.z],Os(t,Wn,Xn,qn,zi))}clampPoint(e,t){return t.copy(e).clamp(this.min,this.max)}distanceToPoint(e){return this.clampPoint(e,Ut).distanceTo(e)}getBoundingSphere(e){return this.isEmpty()?e.makeEmpty():(this.getCenter(e.center),e.radius=this.getSize(Ut).length()*.5),e}intersect(e){return this.min.max(e.min),this.max.min(e.max),this.isEmpty()&&this.makeEmpty(),this}union(e){return this.min.min(e.min),this.max.max(e.max),this}applyMatrix4(e){return this.isEmpty()?this:(qt[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(e),qt[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(e),qt[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(e),qt[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(e),qt[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(e),qt[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(e),qt[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(e),qt[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(e),this.setFromPoints(qt),this)}translate(e){return this.min.add(e),this.max.add(e),this}equals(e){return e.min.equals(this.min)&&e.max.equals(this.max)}}const qt=[new z,new z,new z,new z,new z,new z,new z,new z],Ut=new z,ki=new Di,Wn=new z,Xn=new z,qn=new z,tn=new z,nn=new z,Mn=new z,Si=new z,zi=new z,Vi=new z,bn=new z;function Os(i,e,t,n,r){for(let s=0,o=i.length-3;s<=o;s+=3){bn.fromArray(i,s);const a=r.x*Math.abs(bn.x)+r.y*Math.abs(bn.y)+r.z*Math.abs(bn.z),l=e.dot(bn),c=t.dot(bn),u=n.dot(bn);if(Math.max(-Math.max(l,c,u),Math.min(l,c,u))>a)return!1}return!0}const uf=new Di,Mi=new z,Fs=new z;class Ii{constructor(e=new z,t=-1){this.isSphere=!0,this.center=e,this.radius=t}set(e,t){return this.center.copy(e),this.radius=t,this}setFromPoints(e,t){const n=this.center;t!==void 0?n.copy(t):uf.setFromPoints(e).getCenter(n);let r=0;for(let s=0,o=e.length;s<o;s++)r=Math.max(r,n.distanceToSquared(e[s]));return this.radius=Math.sqrt(r),this}copy(e){return this.center.copy(e.center),this.radius=e.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(e){return e.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(e){return e.distanceTo(this.center)-this.radius}intersectsSphere(e){const t=this.radius+e.radius;return e.center.distanceToSquared(this.center)<=t*t}intersectsBox(e){return e.intersectsSphere(this)}intersectsPlane(e){return Math.abs(e.distanceToPoint(this.center))<=this.radius}clampPoint(e,t){const n=this.center.distanceToSquared(e);return t.copy(e),n>this.radius*this.radius&&(t.sub(this.center).normalize(),t.multiplyScalar(this.radius).add(this.center)),t}getBoundingBox(e){return this.isEmpty()?(e.makeEmpty(),e):(e.set(this.center,this.center),e.expandByScalar(this.radius),e)}applyMatrix4(e){return this.center.applyMatrix4(e),this.radius=this.radius*e.getMaxScaleOnAxis(),this}translate(e){return this.center.add(e),this}expandByPoint(e){if(this.isEmpty())return this.center.copy(e),this.radius=0,this;Mi.subVectors(e,this.center);const t=Mi.lengthSq();if(t>this.radius*this.radius){const n=Math.sqrt(t),r=(n-this.radius)*.5;this.center.addScaledVector(Mi,r/n),this.radius+=r}return this}union(e){return e.isEmpty()?this:this.isEmpty()?(this.copy(e),this):(this.center.equals(e.center)===!0?this.radius=Math.max(this.radius,e.radius):(Fs.subVectors(e.center,this.center).setLength(e.radius),this.expandByPoint(Mi.copy(e.center).add(Fs)),this.expandByPoint(Mi.copy(e.center).sub(Fs))),this)}equals(e){return e.center.equals(this.center)&&e.radius===this.radius}clone(){return new this.constructor().copy(this)}}const Yt=new z,Bs=new z,Gi=new z,rn=new z,ks=new z,Hi=new z,zs=new z;class Ar{constructor(e=new z,t=new z(0,0,-1)){this.origin=e,this.direction=t}set(e,t){return this.origin.copy(e),this.direction.copy(t),this}copy(e){return this.origin.copy(e.origin),this.direction.copy(e.direction),this}at(e,t){return t.copy(this.origin).addScaledVector(this.direction,e)}lookAt(e){return this.direction.copy(e).sub(this.origin).normalize(),this}recast(e){return this.origin.copy(this.at(e,Yt)),this}closestPointToPoint(e,t){t.subVectors(e,this.origin);const n=t.dot(this.direction);return n<0?t.copy(this.origin):t.copy(this.origin).addScaledVector(this.direction,n)}distanceToPoint(e){return Math.sqrt(this.distanceSqToPoint(e))}distanceSqToPoint(e){const t=Yt.subVectors(e,this.origin).dot(this.direction);return t<0?this.origin.distanceToSquared(e):(Yt.copy(this.origin).addScaledVector(this.direction,t),Yt.distanceToSquared(e))}distanceSqToSegment(e,t,n,r){Bs.copy(e).add(t).multiplyScalar(.5),Gi.copy(t).sub(e).normalize(),rn.copy(this.origin).sub(Bs);const s=e.distanceTo(t)*.5,o=-this.direction.dot(Gi),a=rn.dot(this.direction),l=-rn.dot(Gi),c=rn.lengthSq(),u=Math.abs(1-o*o);let h,d,f,g;if(u>0)if(h=o*l-a,d=o*a-l,g=s*u,h>=0)if(d>=-g)if(d<=g){const x=1/u;h*=x,d*=x,f=h*(h+o*d+2*a)+d*(o*h+d+2*l)+c}else d=s,h=Math.max(0,-(o*d+a)),f=-h*h+d*(d+2*l)+c;else d=-s,h=Math.max(0,-(o*d+a)),f=-h*h+d*(d+2*l)+c;else d<=-g?(h=Math.max(0,-(-o*s+a)),d=h>0?-s:Math.min(Math.max(-s,-l),s),f=-h*h+d*(d+2*l)+c):d<=g?(h=0,d=Math.min(Math.max(-s,-l),s),f=d*(d+2*l)+c):(h=Math.max(0,-(o*s+a)),d=h>0?s:Math.min(Math.max(-s,-l),s),f=-h*h+d*(d+2*l)+c);else d=o>0?-s:s,h=Math.max(0,-(o*d+a)),f=-h*h+d*(d+2*l)+c;return n&&n.copy(this.origin).addScaledVector(this.direction,h),r&&r.copy(Bs).addScaledVector(Gi,d),f}intersectSphere(e,t){Yt.subVectors(e.center,this.origin);const n=Yt.dot(this.direction),r=Yt.dot(Yt)-n*n,s=e.radius*e.radius;if(r>s)return null;const o=Math.sqrt(s-r),a=n-o,l=n+o;return l<0?null:a<0?this.at(l,t):this.at(a,t)}intersectsSphere(e){return this.distanceSqToPoint(e.center)<=e.radius*e.radius}distanceToPlane(e){const t=e.normal.dot(this.direction);if(t===0)return e.distanceToPoint(this.origin)===0?0:null;const n=-(this.origin.dot(e.normal)+e.constant)/t;return n>=0?n:null}intersectPlane(e,t){const n=this.distanceToPlane(e);return n===null?null:this.at(n,t)}intersectsPlane(e){const t=e.distanceToPoint(this.origin);return t===0||e.normal.dot(this.direction)*t<0}intersectBox(e,t){let n,r,s,o,a,l;const c=1/this.direction.x,u=1/this.direction.y,h=1/this.direction.z,d=this.origin;return c>=0?(n=(e.min.x-d.x)*c,r=(e.max.x-d.x)*c):(n=(e.max.x-d.x)*c,r=(e.min.x-d.x)*c),u>=0?(s=(e.min.y-d.y)*u,o=(e.max.y-d.y)*u):(s=(e.max.y-d.y)*u,o=(e.min.y-d.y)*u),n>o||s>r||((s>n||isNaN(n))&&(n=s),(o<r||isNaN(r))&&(r=o),h>=0?(a=(e.min.z-d.z)*h,l=(e.max.z-d.z)*h):(a=(e.max.z-d.z)*h,l=(e.min.z-d.z)*h),n>l||a>r)||((a>n||n!==n)&&(n=a),(l<r||r!==r)&&(r=l),r<0)?null:this.at(n>=0?n:r,t)}intersectsBox(e){return this.intersectBox(e,Yt)!==null}intersectTriangle(e,t,n,r,s){ks.subVectors(t,e),Hi.subVectors(n,e),zs.crossVectors(ks,Hi);let o=this.direction.dot(zs),a;if(o>0){if(r)return null;a=1}else if(o<0)a=-1,o=-o;else return null;rn.subVectors(this.origin,e);const l=a*this.direction.dot(Hi.crossVectors(rn,Hi));if(l<0)return null;const c=a*this.direction.dot(ks.cross(rn));if(c<0||l+c>o)return null;const u=-a*rn.dot(zs);return u<0?null:this.at(u/o,s)}applyMatrix4(e){return this.origin.applyMatrix4(e),this.direction.transformDirection(e),this}equals(e){return e.origin.equals(this.origin)&&e.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class Je{constructor(e,t,n,r,s,o,a,l,c,u,h,d,f,g,x,m){Je.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],e!==void 0&&this.set(e,t,n,r,s,o,a,l,c,u,h,d,f,g,x,m)}set(e,t,n,r,s,o,a,l,c,u,h,d,f,g,x,m){const p=this.elements;return p[0]=e,p[4]=t,p[8]=n,p[12]=r,p[1]=s,p[5]=o,p[9]=a,p[13]=l,p[2]=c,p[6]=u,p[10]=h,p[14]=d,p[3]=f,p[7]=g,p[11]=x,p[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new Je().fromArray(this.elements)}copy(e){const t=this.elements,n=e.elements;return t[0]=n[0],t[1]=n[1],t[2]=n[2],t[3]=n[3],t[4]=n[4],t[5]=n[5],t[6]=n[6],t[7]=n[7],t[8]=n[8],t[9]=n[9],t[10]=n[10],t[11]=n[11],t[12]=n[12],t[13]=n[13],t[14]=n[14],t[15]=n[15],this}copyPosition(e){const t=this.elements,n=e.elements;return t[12]=n[12],t[13]=n[13],t[14]=n[14],this}setFromMatrix3(e){const t=e.elements;return this.set(t[0],t[3],t[6],0,t[1],t[4],t[7],0,t[2],t[5],t[8],0,0,0,0,1),this}extractBasis(e,t,n){return e.setFromMatrixColumn(this,0),t.setFromMatrixColumn(this,1),n.setFromMatrixColumn(this,2),this}makeBasis(e,t,n){return this.set(e.x,t.x,n.x,0,e.y,t.y,n.y,0,e.z,t.z,n.z,0,0,0,0,1),this}extractRotation(e){const t=this.elements,n=e.elements,r=1/Yn.setFromMatrixColumn(e,0).length(),s=1/Yn.setFromMatrixColumn(e,1).length(),o=1/Yn.setFromMatrixColumn(e,2).length();return t[0]=n[0]*r,t[1]=n[1]*r,t[2]=n[2]*r,t[3]=0,t[4]=n[4]*s,t[5]=n[5]*s,t[6]=n[6]*s,t[7]=0,t[8]=n[8]*o,t[9]=n[9]*o,t[10]=n[10]*o,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromEuler(e){const t=this.elements,n=e.x,r=e.y,s=e.z,o=Math.cos(n),a=Math.sin(n),l=Math.cos(r),c=Math.sin(r),u=Math.cos(s),h=Math.sin(s);if(e.order==="XYZ"){const d=o*u,f=o*h,g=a*u,x=a*h;t[0]=l*u,t[4]=-l*h,t[8]=c,t[1]=f+g*c,t[5]=d-x*c,t[9]=-a*l,t[2]=x-d*c,t[6]=g+f*c,t[10]=o*l}else if(e.order==="YXZ"){const d=l*u,f=l*h,g=c*u,x=c*h;t[0]=d+x*a,t[4]=g*a-f,t[8]=o*c,t[1]=o*h,t[5]=o*u,t[9]=-a,t[2]=f*a-g,t[6]=x+d*a,t[10]=o*l}else if(e.order==="ZXY"){const d=l*u,f=l*h,g=c*u,x=c*h;t[0]=d-x*a,t[4]=-o*h,t[8]=g+f*a,t[1]=f+g*a,t[5]=o*u,t[9]=x-d*a,t[2]=-o*c,t[6]=a,t[10]=o*l}else if(e.order==="ZYX"){const d=o*u,f=o*h,g=a*u,x=a*h;t[0]=l*u,t[4]=g*c-f,t[8]=d*c+x,t[1]=l*h,t[5]=x*c+d,t[9]=f*c-g,t[2]=-c,t[6]=a*l,t[10]=o*l}else if(e.order==="YZX"){const d=o*l,f=o*c,g=a*l,x=a*c;t[0]=l*u,t[4]=x-d*h,t[8]=g*h+f,t[1]=h,t[5]=o*u,t[9]=-a*u,t[2]=-c*u,t[6]=f*h+g,t[10]=d-x*h}else if(e.order==="XZY"){const d=o*l,f=o*c,g=a*l,x=a*c;t[0]=l*u,t[4]=-h,t[8]=c*u,t[1]=d*h+x,t[5]=o*u,t[9]=f*h-g,t[2]=g*h-f,t[6]=a*u,t[10]=x*h+d}return t[3]=0,t[7]=0,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromQuaternion(e){return this.compose(hf,e,df)}lookAt(e,t,n){const r=this.elements;return Tt.subVectors(e,t),Tt.lengthSq()===0&&(Tt.z=1),Tt.normalize(),sn.crossVectors(n,Tt),sn.lengthSq()===0&&(Math.abs(n.z)===1?Tt.x+=1e-4:Tt.z+=1e-4,Tt.normalize(),sn.crossVectors(n,Tt)),sn.normalize(),Wi.crossVectors(Tt,sn),r[0]=sn.x,r[4]=Wi.x,r[8]=Tt.x,r[1]=sn.y,r[5]=Wi.y,r[9]=Tt.y,r[2]=sn.z,r[6]=Wi.z,r[10]=Tt.z,this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const n=e.elements,r=t.elements,s=this.elements,o=n[0],a=n[4],l=n[8],c=n[12],u=n[1],h=n[5],d=n[9],f=n[13],g=n[2],x=n[6],m=n[10],p=n[14],T=n[3],S=n[7],A=n[11],B=n[15],D=r[0],P=r[4],X=r[8],E=r[12],M=r[1],C=r[5],H=r[9],O=r[13],$=r[2],V=r[6],Y=r[10],te=r[14],v=r[3],w=r[7],U=r[11],N=r[15];return s[0]=o*D+a*M+l*$+c*v,s[4]=o*P+a*C+l*V+c*w,s[8]=o*X+a*H+l*Y+c*U,s[12]=o*E+a*O+l*te+c*N,s[1]=u*D+h*M+d*$+f*v,s[5]=u*P+h*C+d*V+f*w,s[9]=u*X+h*H+d*Y+f*U,s[13]=u*E+h*O+d*te+f*N,s[2]=g*D+x*M+m*$+p*v,s[6]=g*P+x*C+m*V+p*w,s[10]=g*X+x*H+m*Y+p*U,s[14]=g*E+x*O+m*te+p*N,s[3]=T*D+S*M+A*$+B*v,s[7]=T*P+S*C+A*V+B*w,s[11]=T*X+S*H+A*Y+B*U,s[15]=T*E+S*O+A*te+B*N,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[4]*=e,t[8]*=e,t[12]*=e,t[1]*=e,t[5]*=e,t[9]*=e,t[13]*=e,t[2]*=e,t[6]*=e,t[10]*=e,t[14]*=e,t[3]*=e,t[7]*=e,t[11]*=e,t[15]*=e,this}determinant(){const e=this.elements,t=e[0],n=e[4],r=e[8],s=e[12],o=e[1],a=e[5],l=e[9],c=e[13],u=e[2],h=e[6],d=e[10],f=e[14],g=e[3],x=e[7],m=e[11],p=e[15];return g*(+s*l*h-r*c*h-s*a*d+n*c*d+r*a*f-n*l*f)+x*(+t*l*f-t*c*d+s*o*d-r*o*f+r*c*u-s*l*u)+m*(+t*c*h-t*a*f-s*o*h+n*o*f+s*a*u-n*c*u)+p*(-r*a*u-t*l*h+t*a*d+r*o*h-n*o*d+n*l*u)}transpose(){const e=this.elements;let t;return t=e[1],e[1]=e[4],e[4]=t,t=e[2],e[2]=e[8],e[8]=t,t=e[6],e[6]=e[9],e[9]=t,t=e[3],e[3]=e[12],e[12]=t,t=e[7],e[7]=e[13],e[13]=t,t=e[11],e[11]=e[14],e[14]=t,this}setPosition(e,t,n){const r=this.elements;return e.isVector3?(r[12]=e.x,r[13]=e.y,r[14]=e.z):(r[12]=e,r[13]=t,r[14]=n),this}invert(){const e=this.elements,t=e[0],n=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8],h=e[9],d=e[10],f=e[11],g=e[12],x=e[13],m=e[14],p=e[15],T=h*m*c-x*d*c+x*l*f-a*m*f-h*l*p+a*d*p,S=g*d*c-u*m*c-g*l*f+o*m*f+u*l*p-o*d*p,A=u*x*c-g*h*c+g*a*f-o*x*f-u*a*p+o*h*p,B=g*h*l-u*x*l-g*a*d+o*x*d+u*a*m-o*h*m,D=t*T+n*S+r*A+s*B;if(D===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const P=1/D;return e[0]=T*P,e[1]=(x*d*s-h*m*s-x*r*f+n*m*f+h*r*p-n*d*p)*P,e[2]=(a*m*s-x*l*s+x*r*c-n*m*c-a*r*p+n*l*p)*P,e[3]=(h*l*s-a*d*s-h*r*c+n*d*c+a*r*f-n*l*f)*P,e[4]=S*P,e[5]=(u*m*s-g*d*s+g*r*f-t*m*f-u*r*p+t*d*p)*P,e[6]=(g*l*s-o*m*s-g*r*c+t*m*c+o*r*p-t*l*p)*P,e[7]=(o*d*s-u*l*s+u*r*c-t*d*c-o*r*f+t*l*f)*P,e[8]=A*P,e[9]=(g*h*s-u*x*s-g*n*f+t*x*f+u*n*p-t*h*p)*P,e[10]=(o*x*s-g*a*s+g*n*c-t*x*c-o*n*p+t*a*p)*P,e[11]=(u*a*s-o*h*s-u*n*c+t*h*c+o*n*f-t*a*f)*P,e[12]=B*P,e[13]=(u*x*r-g*h*r+g*n*d-t*x*d-u*n*m+t*h*m)*P,e[14]=(g*a*r-o*x*r-g*n*l+t*x*l+o*n*m-t*a*m)*P,e[15]=(o*h*r-u*a*r+u*n*l-t*h*l-o*n*d+t*a*d)*P,this}scale(e){const t=this.elements,n=e.x,r=e.y,s=e.z;return t[0]*=n,t[4]*=r,t[8]*=s,t[1]*=n,t[5]*=r,t[9]*=s,t[2]*=n,t[6]*=r,t[10]*=s,t[3]*=n,t[7]*=r,t[11]*=s,this}getMaxScaleOnAxis(){const e=this.elements,t=e[0]*e[0]+e[1]*e[1]+e[2]*e[2],n=e[4]*e[4]+e[5]*e[5]+e[6]*e[6],r=e[8]*e[8]+e[9]*e[9]+e[10]*e[10];return Math.sqrt(Math.max(t,n,r))}makeTranslation(e,t,n){return e.isVector3?this.set(1,0,0,e.x,0,1,0,e.y,0,0,1,e.z,0,0,0,1):this.set(1,0,0,e,0,1,0,t,0,0,1,n,0,0,0,1),this}makeRotationX(e){const t=Math.cos(e),n=Math.sin(e);return this.set(1,0,0,0,0,t,-n,0,0,n,t,0,0,0,0,1),this}makeRotationY(e){const t=Math.cos(e),n=Math.sin(e);return this.set(t,0,n,0,0,1,0,0,-n,0,t,0,0,0,0,1),this}makeRotationZ(e){const t=Math.cos(e),n=Math.sin(e);return this.set(t,-n,0,0,n,t,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(e,t){const n=Math.cos(t),r=Math.sin(t),s=1-n,o=e.x,a=e.y,l=e.z,c=s*o,u=s*a;return this.set(c*o+n,c*a-r*l,c*l+r*a,0,c*a+r*l,u*a+n,u*l-r*o,0,c*l-r*a,u*l+r*o,s*l*l+n,0,0,0,0,1),this}makeScale(e,t,n){return this.set(e,0,0,0,0,t,0,0,0,0,n,0,0,0,0,1),this}makeShear(e,t,n,r,s,o){return this.set(1,n,s,0,e,1,o,0,t,r,1,0,0,0,0,1),this}compose(e,t,n){const r=this.elements,s=t._x,o=t._y,a=t._z,l=t._w,c=s+s,u=o+o,h=a+a,d=s*c,f=s*u,g=s*h,x=o*u,m=o*h,p=a*h,T=l*c,S=l*u,A=l*h,B=n.x,D=n.y,P=n.z;return r[0]=(1-(x+p))*B,r[1]=(f+A)*B,r[2]=(g-S)*B,r[3]=0,r[4]=(f-A)*D,r[5]=(1-(d+p))*D,r[6]=(m+T)*D,r[7]=0,r[8]=(g+S)*P,r[9]=(m-T)*P,r[10]=(1-(d+x))*P,r[11]=0,r[12]=e.x,r[13]=e.y,r[14]=e.z,r[15]=1,this}decompose(e,t,n){const r=this.elements;let s=Yn.set(r[0],r[1],r[2]).length();const o=Yn.set(r[4],r[5],r[6]).length(),a=Yn.set(r[8],r[9],r[10]).length();this.determinant()<0&&(s=-s),e.x=r[12],e.y=r[13],e.z=r[14],Dt.copy(this);const c=1/s,u=1/o,h=1/a;return Dt.elements[0]*=c,Dt.elements[1]*=c,Dt.elements[2]*=c,Dt.elements[4]*=u,Dt.elements[5]*=u,Dt.elements[6]*=u,Dt.elements[8]*=h,Dt.elements[9]*=h,Dt.elements[10]*=h,t.setFromRotationMatrix(Dt),n.x=s,n.y=o,n.z=a,this}makePerspective(e,t,n,r,s,o,a=Jt){const l=this.elements,c=2*s/(t-e),u=2*s/(n-r),h=(t+e)/(t-e),d=(n+r)/(n-r);let f,g;if(a===Jt)f=-(o+s)/(o-s),g=-2*o*s/(o-s);else if(a===xr)f=-o/(o-s),g=-o*s/(o-s);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+a);return l[0]=c,l[4]=0,l[8]=h,l[12]=0,l[1]=0,l[5]=u,l[9]=d,l[13]=0,l[2]=0,l[6]=0,l[10]=f,l[14]=g,l[3]=0,l[7]=0,l[11]=-1,l[15]=0,this}makeOrthographic(e,t,n,r,s,o,a=Jt){const l=this.elements,c=1/(t-e),u=1/(n-r),h=1/(o-s),d=(t+e)*c,f=(n+r)*u;let g,x;if(a===Jt)g=(o+s)*h,x=-2*h;else if(a===xr)g=s*h,x=-1*h;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+a);return l[0]=2*c,l[4]=0,l[8]=0,l[12]=-d,l[1]=0,l[5]=2*u,l[9]=0,l[13]=-f,l[2]=0,l[6]=0,l[10]=x,l[14]=-g,l[3]=0,l[7]=0,l[11]=0,l[15]=1,this}equals(e){const t=this.elements,n=e.elements;for(let r=0;r<16;r++)if(t[r]!==n[r])return!1;return!0}fromArray(e,t=0){for(let n=0;n<16;n++)this.elements[n]=e[n+t];return this}toArray(e=[],t=0){const n=this.elements;return e[t]=n[0],e[t+1]=n[1],e[t+2]=n[2],e[t+3]=n[3],e[t+4]=n[4],e[t+5]=n[5],e[t+6]=n[6],e[t+7]=n[7],e[t+8]=n[8],e[t+9]=n[9],e[t+10]=n[10],e[t+11]=n[11],e[t+12]=n[12],e[t+13]=n[13],e[t+14]=n[14],e[t+15]=n[15],e}}const Yn=new z,Dt=new Je,hf=new z(0,0,0),df=new z(1,1,1),sn=new z,Wi=new z,Tt=new z,gl=new Je,_l=new Ht;class Wt{constructor(e=0,t=0,n=0,r=Wt.DEFAULT_ORDER){this.isEuler=!0,this._x=e,this._y=t,this._z=n,this._order=r}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get order(){return this._order}set order(e){this._order=e,this._onChangeCallback()}set(e,t,n,r=this._order){return this._x=e,this._y=t,this._z=n,this._order=r,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(e){return this._x=e._x,this._y=e._y,this._z=e._z,this._order=e._order,this._onChangeCallback(),this}setFromRotationMatrix(e,t=this._order,n=!0){const r=e.elements,s=r[0],o=r[4],a=r[8],l=r[1],c=r[5],u=r[9],h=r[2],d=r[6],f=r[10];switch(t){case"XYZ":this._y=Math.asin(vt(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(-u,f),this._z=Math.atan2(-o,s)):(this._x=Math.atan2(d,c),this._z=0);break;case"YXZ":this._x=Math.asin(-vt(u,-1,1)),Math.abs(u)<.9999999?(this._y=Math.atan2(a,f),this._z=Math.atan2(l,c)):(this._y=Math.atan2(-h,s),this._z=0);break;case"ZXY":this._x=Math.asin(vt(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-h,f),this._z=Math.atan2(-o,c)):(this._y=0,this._z=Math.atan2(l,s));break;case"ZYX":this._y=Math.asin(-vt(h,-1,1)),Math.abs(h)<.9999999?(this._x=Math.atan2(d,f),this._z=Math.atan2(l,s)):(this._x=0,this._z=Math.atan2(-o,c));break;case"YZX":this._z=Math.asin(vt(l,-1,1)),Math.abs(l)<.9999999?(this._x=Math.atan2(-u,c),this._y=Math.atan2(-h,s)):(this._x=0,this._y=Math.atan2(a,f));break;case"XZY":this._z=Math.asin(-vt(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(d,c),this._y=Math.atan2(a,s)):(this._x=Math.atan2(-u,f),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+t)}return this._order=t,n===!0&&this._onChangeCallback(),this}setFromQuaternion(e,t,n){return gl.makeRotationFromQuaternion(e),this.setFromRotationMatrix(gl,t,n)}setFromVector3(e,t=this._order){return this.set(e.x,e.y,e.z,t)}reorder(e){return _l.setFromEuler(this),this.setFromQuaternion(_l,e)}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._order===this._order}fromArray(e){return this._x=e[0],this._y=e[1],this._z=e[2],e[3]!==void 0&&(this._order=e[3]),this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._order,e}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}Wt.DEFAULT_ORDER="XYZ";class Zc{constructor(){this.mask=1}set(e){this.mask=(1<<e|0)>>>0}enable(e){this.mask|=1<<e|0}enableAll(){this.mask=-1}toggle(e){this.mask^=1<<e|0}disable(e){this.mask&=~(1<<e|0)}disableAll(){this.mask=0}test(e){return(this.mask&e.mask)!==0}isEnabled(e){return(this.mask&(1<<e|0))!==0}}let ff=0;const vl=new z,jn=new Ht,jt=new Je,Xi=new z,bi=new z,pf=new z,mf=new Ht,xl=new z(1,0,0),yl=new z(0,1,0),Sl=new z(0,0,1),Ml={type:"added"},gf={type:"removed"},$n={type:"childadded",child:null},Vs={type:"childremoved",child:null};class dt extends Bn{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:ff++}),this.uuid=Ui(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=dt.DEFAULT_UP.clone();const e=new z,t=new Wt,n=new Ht,r=new z(1,1,1);function s(){n.setFromEuler(t,!1)}function o(){t.setFromQuaternion(n,void 0,!1)}t._onChange(s),n._onChange(o),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:e},rotation:{configurable:!0,enumerable:!0,value:t},quaternion:{configurable:!0,enumerable:!0,value:n},scale:{configurable:!0,enumerable:!0,value:r},modelViewMatrix:{value:new Je},normalMatrix:{value:new ze}}),this.matrix=new Je,this.matrixWorld=new Je,this.matrixAutoUpdate=dt.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=dt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new Zc,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(e){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(e),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(e){return this.quaternion.premultiply(e),this}setRotationFromAxisAngle(e,t){this.quaternion.setFromAxisAngle(e,t)}setRotationFromEuler(e){this.quaternion.setFromEuler(e,!0)}setRotationFromMatrix(e){this.quaternion.setFromRotationMatrix(e)}setRotationFromQuaternion(e){this.quaternion.copy(e)}rotateOnAxis(e,t){return jn.setFromAxisAngle(e,t),this.quaternion.multiply(jn),this}rotateOnWorldAxis(e,t){return jn.setFromAxisAngle(e,t),this.quaternion.premultiply(jn),this}rotateX(e){return this.rotateOnAxis(xl,e)}rotateY(e){return this.rotateOnAxis(yl,e)}rotateZ(e){return this.rotateOnAxis(Sl,e)}translateOnAxis(e,t){return vl.copy(e).applyQuaternion(this.quaternion),this.position.add(vl.multiplyScalar(t)),this}translateX(e){return this.translateOnAxis(xl,e)}translateY(e){return this.translateOnAxis(yl,e)}translateZ(e){return this.translateOnAxis(Sl,e)}localToWorld(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(this.matrixWorld)}worldToLocal(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(jt.copy(this.matrixWorld).invert())}lookAt(e,t,n){e.isVector3?Xi.copy(e):Xi.set(e,t,n);const r=this.parent;this.updateWorldMatrix(!0,!1),bi.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?jt.lookAt(bi,Xi,this.up):jt.lookAt(Xi,bi,this.up),this.quaternion.setFromRotationMatrix(jt),r&&(jt.extractRotation(r.matrixWorld),jn.setFromRotationMatrix(jt),this.quaternion.premultiply(jn.invert()))}add(e){if(arguments.length>1){for(let t=0;t<arguments.length;t++)this.add(arguments[t]);return this}return e===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",e),this):(e&&e.isObject3D?(e.removeFromParent(),e.parent=this,this.children.push(e),e.dispatchEvent(Ml),$n.child=e,this.dispatchEvent($n),$n.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",e),this)}remove(e){if(arguments.length>1){for(let n=0;n<arguments.length;n++)this.remove(arguments[n]);return this}const t=this.children.indexOf(e);return t!==-1&&(e.parent=null,this.children.splice(t,1),e.dispatchEvent(gf),Vs.child=e,this.dispatchEvent(Vs),Vs.child=null),this}removeFromParent(){const e=this.parent;return e!==null&&e.remove(this),this}clear(){return this.remove(...this.children)}attach(e){return this.updateWorldMatrix(!0,!1),jt.copy(this.matrixWorld).invert(),e.parent!==null&&(e.parent.updateWorldMatrix(!0,!1),jt.multiply(e.parent.matrixWorld)),e.applyMatrix4(jt),e.removeFromParent(),e.parent=this,this.children.push(e),e.updateWorldMatrix(!1,!0),e.dispatchEvent(Ml),$n.child=e,this.dispatchEvent($n),$n.child=null,this}getObjectById(e){return this.getObjectByProperty("id",e)}getObjectByName(e){return this.getObjectByProperty("name",e)}getObjectByProperty(e,t){if(this[e]===t)return this;for(let n=0,r=this.children.length;n<r;n++){const o=this.children[n].getObjectByProperty(e,t);if(o!==void 0)return o}}getObjectsByProperty(e,t,n=[]){this[e]===t&&n.push(this);const r=this.children;for(let s=0,o=r.length;s<o;s++)r[s].getObjectsByProperty(e,t,n);return n}getWorldPosition(e){return this.updateWorldMatrix(!0,!1),e.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(bi,e,pf),e}getWorldScale(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(bi,mf,e),e}getWorldDirection(e){this.updateWorldMatrix(!0,!1);const t=this.matrixWorld.elements;return e.set(t[8],t[9],t[10]).normalize()}raycast(){}traverse(e){e(this);const t=this.children;for(let n=0,r=t.length;n<r;n++)t[n].traverse(e)}traverseVisible(e){if(this.visible===!1)return;e(this);const t=this.children;for(let n=0,r=t.length;n<r;n++)t[n].traverseVisible(e)}traverseAncestors(e){const t=this.parent;t!==null&&(e(t),t.traverseAncestors(e))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(e){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||e)&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),this.matrixWorldNeedsUpdate=!1,e=!0);const t=this.children;for(let n=0,r=t.length;n<r;n++){const s=t[n];(s.matrixWorldAutoUpdate===!0||e===!0)&&s.updateMatrixWorld(e)}}updateWorldMatrix(e,t){const n=this.parent;if(e===!0&&n!==null&&n.matrixWorldAutoUpdate===!0&&n.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),t===!0){const r=this.children;for(let s=0,o=r.length;s<o;s++){const a=r[s];a.matrixWorldAutoUpdate===!0&&a.updateWorldMatrix(!1,!0)}}}toJSON(e){const t=e===void 0||typeof e=="string",n={};t&&(e={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},n.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const r={};r.uuid=this.uuid,r.type=this.type,this.name!==""&&(r.name=this.name),this.castShadow===!0&&(r.castShadow=!0),this.receiveShadow===!0&&(r.receiveShadow=!0),this.visible===!1&&(r.visible=!1),this.frustumCulled===!1&&(r.frustumCulled=!1),this.renderOrder!==0&&(r.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(r.userData=this.userData),r.layers=this.layers.mask,r.matrix=this.matrix.toArray(),r.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(r.matrixAutoUpdate=!1),this.isInstancedMesh&&(r.type="InstancedMesh",r.count=this.count,r.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(r.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(r.type="BatchedMesh",r.perObjectFrustumCulled=this.perObjectFrustumCulled,r.sortObjects=this.sortObjects,r.drawRanges=this._drawRanges,r.reservedRanges=this._reservedRanges,r.visibility=this._visibility,r.active=this._active,r.bounds=this._bounds.map(a=>({boxInitialized:a.boxInitialized,boxMin:a.box.min.toArray(),boxMax:a.box.max.toArray(),sphereInitialized:a.sphereInitialized,sphereRadius:a.sphere.radius,sphereCenter:a.sphere.center.toArray()})),r.maxGeometryCount=this._maxGeometryCount,r.maxVertexCount=this._maxVertexCount,r.maxIndexCount=this._maxIndexCount,r.geometryInitialized=this._geometryInitialized,r.geometryCount=this._geometryCount,r.matricesTexture=this._matricesTexture.toJSON(e),this._colorsTexture!==null&&(r.colorsTexture=this._colorsTexture.toJSON(e)),this.boundingSphere!==null&&(r.boundingSphere={center:r.boundingSphere.center.toArray(),radius:r.boundingSphere.radius}),this.boundingBox!==null&&(r.boundingBox={min:r.boundingBox.min.toArray(),max:r.boundingBox.max.toArray()}));function s(a,l){return a[l.uuid]===void 0&&(a[l.uuid]=l.toJSON(e)),l.uuid}if(this.isScene)this.background&&(this.background.isColor?r.background=this.background.toJSON():this.background.isTexture&&(r.background=this.background.toJSON(e).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(r.environment=this.environment.toJSON(e).uuid);else if(this.isMesh||this.isLine||this.isPoints){r.geometry=s(e.geometries,this.geometry);const a=this.geometry.parameters;if(a!==void 0&&a.shapes!==void 0){const l=a.shapes;if(Array.isArray(l))for(let c=0,u=l.length;c<u;c++){const h=l[c];s(e.shapes,h)}else s(e.shapes,l)}}if(this.isSkinnedMesh&&(r.bindMode=this.bindMode,r.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(s(e.skeletons,this.skeleton),r.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const a=[];for(let l=0,c=this.material.length;l<c;l++)a.push(s(e.materials,this.material[l]));r.material=a}else r.material=s(e.materials,this.material);if(this.children.length>0){r.children=[];for(let a=0;a<this.children.length;a++)r.children.push(this.children[a].toJSON(e).object)}if(this.animations.length>0){r.animations=[];for(let a=0;a<this.animations.length;a++){const l=this.animations[a];r.animations.push(s(e.animations,l))}}if(t){const a=o(e.geometries),l=o(e.materials),c=o(e.textures),u=o(e.images),h=o(e.shapes),d=o(e.skeletons),f=o(e.animations),g=o(e.nodes);a.length>0&&(n.geometries=a),l.length>0&&(n.materials=l),c.length>0&&(n.textures=c),u.length>0&&(n.images=u),h.length>0&&(n.shapes=h),d.length>0&&(n.skeletons=d),f.length>0&&(n.animations=f),g.length>0&&(n.nodes=g)}return n.object=r,n;function o(a){const l=[];for(const c in a){const u=a[c];delete u.metadata,l.push(u)}return l}}clone(e){return new this.constructor().copy(this,e)}copy(e,t=!0){if(this.name=e.name,this.up.copy(e.up),this.position.copy(e.position),this.rotation.order=e.rotation.order,this.quaternion.copy(e.quaternion),this.scale.copy(e.scale),this.matrix.copy(e.matrix),this.matrixWorld.copy(e.matrixWorld),this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrixWorldAutoUpdate=e.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=e.matrixWorldNeedsUpdate,this.layers.mask=e.layers.mask,this.visible=e.visible,this.castShadow=e.castShadow,this.receiveShadow=e.receiveShadow,this.frustumCulled=e.frustumCulled,this.renderOrder=e.renderOrder,this.animations=e.animations.slice(),this.userData=JSON.parse(JSON.stringify(e.userData)),t===!0)for(let n=0;n<e.children.length;n++){const r=e.children[n];this.add(r.clone())}return this}}dt.DEFAULT_UP=new z(0,1,0);dt.DEFAULT_MATRIX_AUTO_UPDATE=!0;dt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const It=new z,$t=new z,Gs=new z,Kt=new z,Kn=new z,Zn=new z,bl=new z,Hs=new z,Ws=new z,Xs=new z;class zt{constructor(e=new z,t=new z,n=new z){this.a=e,this.b=t,this.c=n}static getNormal(e,t,n,r){r.subVectors(n,t),It.subVectors(e,t),r.cross(It);const s=r.lengthSq();return s>0?r.multiplyScalar(1/Math.sqrt(s)):r.set(0,0,0)}static getBarycoord(e,t,n,r,s){It.subVectors(r,t),$t.subVectors(n,t),Gs.subVectors(e,t);const o=It.dot(It),a=It.dot($t),l=It.dot(Gs),c=$t.dot($t),u=$t.dot(Gs),h=o*c-a*a;if(h===0)return s.set(0,0,0),null;const d=1/h,f=(c*l-a*u)*d,g=(o*u-a*l)*d;return s.set(1-f-g,g,f)}static containsPoint(e,t,n,r){return this.getBarycoord(e,t,n,r,Kt)===null?!1:Kt.x>=0&&Kt.y>=0&&Kt.x+Kt.y<=1}static getInterpolation(e,t,n,r,s,o,a,l){return this.getBarycoord(e,t,n,r,Kt)===null?(l.x=0,l.y=0,"z"in l&&(l.z=0),"w"in l&&(l.w=0),null):(l.setScalar(0),l.addScaledVector(s,Kt.x),l.addScaledVector(o,Kt.y),l.addScaledVector(a,Kt.z),l)}static isFrontFacing(e,t,n,r){return It.subVectors(n,t),$t.subVectors(e,t),It.cross($t).dot(r)<0}set(e,t,n){return this.a.copy(e),this.b.copy(t),this.c.copy(n),this}setFromPointsAndIndices(e,t,n,r){return this.a.copy(e[t]),this.b.copy(e[n]),this.c.copy(e[r]),this}setFromAttributeAndIndices(e,t,n,r){return this.a.fromBufferAttribute(e,t),this.b.fromBufferAttribute(e,n),this.c.fromBufferAttribute(e,r),this}clone(){return new this.constructor().copy(this)}copy(e){return this.a.copy(e.a),this.b.copy(e.b),this.c.copy(e.c),this}getArea(){return It.subVectors(this.c,this.b),$t.subVectors(this.a,this.b),It.cross($t).length()*.5}getMidpoint(e){return e.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(e){return zt.getNormal(this.a,this.b,this.c,e)}getPlane(e){return e.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(e,t){return zt.getBarycoord(e,this.a,this.b,this.c,t)}getInterpolation(e,t,n,r,s){return zt.getInterpolation(e,this.a,this.b,this.c,t,n,r,s)}containsPoint(e){return zt.containsPoint(e,this.a,this.b,this.c)}isFrontFacing(e){return zt.isFrontFacing(this.a,this.b,this.c,e)}intersectsBox(e){return e.intersectsTriangle(this)}closestPointToPoint(e,t){const n=this.a,r=this.b,s=this.c;let o,a;Kn.subVectors(r,n),Zn.subVectors(s,n),Hs.subVectors(e,n);const l=Kn.dot(Hs),c=Zn.dot(Hs);if(l<=0&&c<=0)return t.copy(n);Ws.subVectors(e,r);const u=Kn.dot(Ws),h=Zn.dot(Ws);if(u>=0&&h<=u)return t.copy(r);const d=l*h-u*c;if(d<=0&&l>=0&&u<=0)return o=l/(l-u),t.copy(n).addScaledVector(Kn,o);Xs.subVectors(e,s);const f=Kn.dot(Xs),g=Zn.dot(Xs);if(g>=0&&f<=g)return t.copy(s);const x=f*c-l*g;if(x<=0&&c>=0&&g<=0)return a=c/(c-g),t.copy(n).addScaledVector(Zn,a);const m=u*g-f*h;if(m<=0&&h-u>=0&&f-g>=0)return bl.subVectors(s,r),a=(h-u)/(h-u+(f-g)),t.copy(r).addScaledVector(bl,a);const p=1/(m+x+d);return o=x*p,a=d*p,t.copy(n).addScaledVector(Kn,o).addScaledVector(Zn,a)}equals(e){return e.a.equals(this.a)&&e.b.equals(this.b)&&e.c.equals(this.c)}}const Jc={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},an={h:0,s:0,l:0},qi={h:0,s:0,l:0};function qs(i,e,t){return t<0&&(t+=1),t>1&&(t-=1),t<1/6?i+(e-i)*6*t:t<1/2?e:t<2/3?i+(e-i)*6*(2/3-t):i}class He{constructor(e,t,n){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(e,t,n)}set(e,t,n){if(t===void 0&&n===void 0){const r=e;r&&r.isColor?this.copy(r):typeof r=="number"?this.setHex(r):typeof r=="string"&&this.setStyle(r)}else this.setRGB(e,t,n);return this}setScalar(e){return this.r=e,this.g=e,this.b=e,this}setHex(e,t=Bt){return e=Math.floor(e),this.r=(e>>16&255)/255,this.g=(e>>8&255)/255,this.b=(e&255)/255,je.toWorkingColorSpace(this,t),this}setRGB(e,t,n,r=je.workingColorSpace){return this.r=e,this.g=t,this.b=n,je.toWorkingColorSpace(this,r),this}setHSL(e,t,n,r=je.workingColorSpace){if(e=Qd(e,1),t=vt(t,0,1),n=vt(n,0,1),t===0)this.r=this.g=this.b=n;else{const s=n<=.5?n*(1+t):n+t-n*t,o=2*n-s;this.r=qs(o,s,e+1/3),this.g=qs(o,s,e),this.b=qs(o,s,e-1/3)}return je.toWorkingColorSpace(this,r),this}setStyle(e,t=Bt){function n(s){s!==void 0&&parseFloat(s)<1&&console.warn("THREE.Color: Alpha component of "+e+" will be ignored.")}let r;if(r=/^(\w+)\(([^\)]*)\)/.exec(e)){let s;const o=r[1],a=r[2];switch(o){case"rgb":case"rgba":if(s=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(s[4]),this.setRGB(Math.min(255,parseInt(s[1],10))/255,Math.min(255,parseInt(s[2],10))/255,Math.min(255,parseInt(s[3],10))/255,t);if(s=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(s[4]),this.setRGB(Math.min(100,parseInt(s[1],10))/100,Math.min(100,parseInt(s[2],10))/100,Math.min(100,parseInt(s[3],10))/100,t);break;case"hsl":case"hsla":if(s=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(s[4]),this.setHSL(parseFloat(s[1])/360,parseFloat(s[2])/100,parseFloat(s[3])/100,t);break;default:console.warn("THREE.Color: Unknown color model "+e)}}else if(r=/^\#([A-Fa-f\d]+)$/.exec(e)){const s=r[1],o=s.length;if(o===3)return this.setRGB(parseInt(s.charAt(0),16)/15,parseInt(s.charAt(1),16)/15,parseInt(s.charAt(2),16)/15,t);if(o===6)return this.setHex(parseInt(s,16),t);console.warn("THREE.Color: Invalid hex color "+e)}else if(e&&e.length>0)return this.setColorName(e,t);return this}setColorName(e,t=Bt){const n=Jc[e.toLowerCase()];return n!==void 0?this.setHex(n,t):console.warn("THREE.Color: Unknown color "+e),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(e){return this.r=e.r,this.g=e.g,this.b=e.b,this}copySRGBToLinear(e){return this.r=ci(e.r),this.g=ci(e.g),this.b=ci(e.b),this}copyLinearToSRGB(e){return this.r=Ds(e.r),this.g=Ds(e.g),this.b=Ds(e.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(e=Bt){return je.fromWorkingColorSpace(mt.copy(this),e),Math.round(vt(mt.r*255,0,255))*65536+Math.round(vt(mt.g*255,0,255))*256+Math.round(vt(mt.b*255,0,255))}getHexString(e=Bt){return("000000"+this.getHex(e).toString(16)).slice(-6)}getHSL(e,t=je.workingColorSpace){je.fromWorkingColorSpace(mt.copy(this),t);const n=mt.r,r=mt.g,s=mt.b,o=Math.max(n,r,s),a=Math.min(n,r,s);let l,c;const u=(a+o)/2;if(a===o)l=0,c=0;else{const h=o-a;switch(c=u<=.5?h/(o+a):h/(2-o-a),o){case n:l=(r-s)/h+(r<s?6:0);break;case r:l=(s-n)/h+2;break;case s:l=(n-r)/h+4;break}l/=6}return e.h=l,e.s=c,e.l=u,e}getRGB(e,t=je.workingColorSpace){return je.fromWorkingColorSpace(mt.copy(this),t),e.r=mt.r,e.g=mt.g,e.b=mt.b,e}getStyle(e=Bt){je.fromWorkingColorSpace(mt.copy(this),e);const t=mt.r,n=mt.g,r=mt.b;return e!==Bt?`color(${e} ${t.toFixed(3)} ${n.toFixed(3)} ${r.toFixed(3)})`:`rgb(${Math.round(t*255)},${Math.round(n*255)},${Math.round(r*255)})`}offsetHSL(e,t,n){return this.getHSL(an),this.setHSL(an.h+e,an.s+t,an.l+n)}add(e){return this.r+=e.r,this.g+=e.g,this.b+=e.b,this}addColors(e,t){return this.r=e.r+t.r,this.g=e.g+t.g,this.b=e.b+t.b,this}addScalar(e){return this.r+=e,this.g+=e,this.b+=e,this}sub(e){return this.r=Math.max(0,this.r-e.r),this.g=Math.max(0,this.g-e.g),this.b=Math.max(0,this.b-e.b),this}multiply(e){return this.r*=e.r,this.g*=e.g,this.b*=e.b,this}multiplyScalar(e){return this.r*=e,this.g*=e,this.b*=e,this}lerp(e,t){return this.r+=(e.r-this.r)*t,this.g+=(e.g-this.g)*t,this.b+=(e.b-this.b)*t,this}lerpColors(e,t,n){return this.r=e.r+(t.r-e.r)*n,this.g=e.g+(t.g-e.g)*n,this.b=e.b+(t.b-e.b)*n,this}lerpHSL(e,t){this.getHSL(an),e.getHSL(qi);const n=Ls(an.h,qi.h,t),r=Ls(an.s,qi.s,t),s=Ls(an.l,qi.l,t);return this.setHSL(n,r,s),this}setFromVector3(e){return this.r=e.x,this.g=e.y,this.b=e.z,this}applyMatrix3(e){const t=this.r,n=this.g,r=this.b,s=e.elements;return this.r=s[0]*t+s[3]*n+s[6]*r,this.g=s[1]*t+s[4]*n+s[7]*r,this.b=s[2]*t+s[5]*n+s[8]*r,this}equals(e){return e.r===this.r&&e.g===this.g&&e.b===this.b}fromArray(e,t=0){return this.r=e[t],this.g=e[t+1],this.b=e[t+2],this}toArray(e=[],t=0){return e[t]=this.r,e[t+1]=this.g,e[t+2]=this.b,e}fromBufferAttribute(e,t){return this.r=e.getX(t),this.g=e.getY(t),this.b=e.getZ(t),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const mt=new He;He.NAMES=Jc;let _f=0;class kn extends Bn{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:_f++}),this.uuid=Ui(),this.name="",this.type="Material",this.blending=oi,this.side=mn,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=ha,this.blendDst=da,this.blendEquation=Rn,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new He(0,0,0),this.blendAlpha=0,this.depthFunc=pr,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=cl,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=Gn,this.stencilZFail=Gn,this.stencilZPass=Gn,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(e){this._alphaTest>0!=e>0&&this.version++,this._alphaTest=e}onBuild(){}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(e){if(e!==void 0)for(const t in e){const n=e[t];if(n===void 0){console.warn(`THREE.Material: parameter '${t}' has value of undefined.`);continue}const r=this[t];if(r===void 0){console.warn(`THREE.Material: '${t}' is not a property of THREE.${this.type}.`);continue}r&&r.isColor?r.set(n):r&&r.isVector3&&n&&n.isVector3?r.copy(n):this[t]=n}}toJSON(e){const t=e===void 0||typeof e=="string";t&&(e={textures:{},images:{}});const n={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};n.uuid=this.uuid,n.type=this.type,this.name!==""&&(n.name=this.name),this.color&&this.color.isColor&&(n.color=this.color.getHex()),this.roughness!==void 0&&(n.roughness=this.roughness),this.metalness!==void 0&&(n.metalness=this.metalness),this.sheen!==void 0&&(n.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(n.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(n.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(n.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(n.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(n.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(n.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(n.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(n.shininess=this.shininess),this.clearcoat!==void 0&&(n.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(n.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(n.clearcoatMap=this.clearcoatMap.toJSON(e).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(n.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(e).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(n.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(e).uuid,n.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.dispersion!==void 0&&(n.dispersion=this.dispersion),this.iridescence!==void 0&&(n.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(n.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(n.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(n.iridescenceMap=this.iridescenceMap.toJSON(e).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(n.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(e).uuid),this.anisotropy!==void 0&&(n.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(n.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(n.anisotropyMap=this.anisotropyMap.toJSON(e).uuid),this.map&&this.map.isTexture&&(n.map=this.map.toJSON(e).uuid),this.matcap&&this.matcap.isTexture&&(n.matcap=this.matcap.toJSON(e).uuid),this.alphaMap&&this.alphaMap.isTexture&&(n.alphaMap=this.alphaMap.toJSON(e).uuid),this.lightMap&&this.lightMap.isTexture&&(n.lightMap=this.lightMap.toJSON(e).uuid,n.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(n.aoMap=this.aoMap.toJSON(e).uuid,n.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(n.bumpMap=this.bumpMap.toJSON(e).uuid,n.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(n.normalMap=this.normalMap.toJSON(e).uuid,n.normalMapType=this.normalMapType,n.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(n.displacementMap=this.displacementMap.toJSON(e).uuid,n.displacementScale=this.displacementScale,n.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(n.roughnessMap=this.roughnessMap.toJSON(e).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(n.metalnessMap=this.metalnessMap.toJSON(e).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(n.emissiveMap=this.emissiveMap.toJSON(e).uuid),this.specularMap&&this.specularMap.isTexture&&(n.specularMap=this.specularMap.toJSON(e).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(n.specularIntensityMap=this.specularIntensityMap.toJSON(e).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(n.specularColorMap=this.specularColorMap.toJSON(e).uuid),this.envMap&&this.envMap.isTexture&&(n.envMap=this.envMap.toJSON(e).uuid,this.combine!==void 0&&(n.combine=this.combine)),this.envMapRotation!==void 0&&(n.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(n.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(n.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(n.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(n.gradientMap=this.gradientMap.toJSON(e).uuid),this.transmission!==void 0&&(n.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(n.transmissionMap=this.transmissionMap.toJSON(e).uuid),this.thickness!==void 0&&(n.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(n.thicknessMap=this.thicknessMap.toJSON(e).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(n.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(n.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(n.size=this.size),this.shadowSide!==null&&(n.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(n.sizeAttenuation=this.sizeAttenuation),this.blending!==oi&&(n.blending=this.blending),this.side!==mn&&(n.side=this.side),this.vertexColors===!0&&(n.vertexColors=!0),this.opacity<1&&(n.opacity=this.opacity),this.transparent===!0&&(n.transparent=!0),this.blendSrc!==ha&&(n.blendSrc=this.blendSrc),this.blendDst!==da&&(n.blendDst=this.blendDst),this.blendEquation!==Rn&&(n.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(n.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(n.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(n.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(n.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(n.blendAlpha=this.blendAlpha),this.depthFunc!==pr&&(n.depthFunc=this.depthFunc),this.depthTest===!1&&(n.depthTest=this.depthTest),this.depthWrite===!1&&(n.depthWrite=this.depthWrite),this.colorWrite===!1&&(n.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(n.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==cl&&(n.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(n.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(n.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==Gn&&(n.stencilFail=this.stencilFail),this.stencilZFail!==Gn&&(n.stencilZFail=this.stencilZFail),this.stencilZPass!==Gn&&(n.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(n.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(n.rotation=this.rotation),this.polygonOffset===!0&&(n.polygonOffset=!0),this.polygonOffsetFactor!==0&&(n.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(n.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(n.linewidth=this.linewidth),this.dashSize!==void 0&&(n.dashSize=this.dashSize),this.gapSize!==void 0&&(n.gapSize=this.gapSize),this.scale!==void 0&&(n.scale=this.scale),this.dithering===!0&&(n.dithering=!0),this.alphaTest>0&&(n.alphaTest=this.alphaTest),this.alphaHash===!0&&(n.alphaHash=!0),this.alphaToCoverage===!0&&(n.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(n.premultipliedAlpha=!0),this.forceSinglePass===!0&&(n.forceSinglePass=!0),this.wireframe===!0&&(n.wireframe=!0),this.wireframeLinewidth>1&&(n.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(n.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(n.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(n.flatShading=!0),this.visible===!1&&(n.visible=!1),this.toneMapped===!1&&(n.toneMapped=!1),this.fog===!1&&(n.fog=!1),Object.keys(this.userData).length>0&&(n.userData=this.userData);function r(s){const o=[];for(const a in s){const l=s[a];delete l.metadata,o.push(l)}return o}if(t){const s=r(e.textures),o=r(e.images);s.length>0&&(n.textures=s),o.length>0&&(n.images=o)}return n}clone(){return new this.constructor().copy(this)}copy(e){this.name=e.name,this.blending=e.blending,this.side=e.side,this.vertexColors=e.vertexColors,this.opacity=e.opacity,this.transparent=e.transparent,this.blendSrc=e.blendSrc,this.blendDst=e.blendDst,this.blendEquation=e.blendEquation,this.blendSrcAlpha=e.blendSrcAlpha,this.blendDstAlpha=e.blendDstAlpha,this.blendEquationAlpha=e.blendEquationAlpha,this.blendColor.copy(e.blendColor),this.blendAlpha=e.blendAlpha,this.depthFunc=e.depthFunc,this.depthTest=e.depthTest,this.depthWrite=e.depthWrite,this.stencilWriteMask=e.stencilWriteMask,this.stencilFunc=e.stencilFunc,this.stencilRef=e.stencilRef,this.stencilFuncMask=e.stencilFuncMask,this.stencilFail=e.stencilFail,this.stencilZFail=e.stencilZFail,this.stencilZPass=e.stencilZPass,this.stencilWrite=e.stencilWrite;const t=e.clippingPlanes;let n=null;if(t!==null){const r=t.length;n=new Array(r);for(let s=0;s!==r;++s)n[s]=t[s].clone()}return this.clippingPlanes=n,this.clipIntersection=e.clipIntersection,this.clipShadows=e.clipShadows,this.shadowSide=e.shadowSide,this.colorWrite=e.colorWrite,this.precision=e.precision,this.polygonOffset=e.polygonOffset,this.polygonOffsetFactor=e.polygonOffsetFactor,this.polygonOffsetUnits=e.polygonOffsetUnits,this.dithering=e.dithering,this.alphaTest=e.alphaTest,this.alphaHash=e.alphaHash,this.alphaToCoverage=e.alphaToCoverage,this.premultipliedAlpha=e.premultipliedAlpha,this.forceSinglePass=e.forceSinglePass,this.visible=e.visible,this.toneMapped=e.toneMapped,this.userData=JSON.parse(JSON.stringify(e.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(e){e===!0&&this.version++}}class Qc extends kn{constructor(e){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new He(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Wt,this.combine=Fc,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.specularMap=e.specularMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.combine=e.combine,this.reflectivity=e.reflectivity,this.refractionRatio=e.refractionRatio,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.fog=e.fog,this}}const at=new z,Yi=new De;class Gt{constructor(e,t,n=!1){if(Array.isArray(e))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=e,this.itemSize=t,this.count=e!==void 0?e.length/t:0,this.normalized=n,this.usage=ul,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=hn,this.version=0}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}get updateRange(){return jc("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.name=e.name,this.array=new e.array.constructor(e.array),this.itemSize=e.itemSize,this.count=e.count,this.normalized=e.normalized,this.usage=e.usage,this.gpuType=e.gpuType,this}copyAt(e,t,n){e*=this.itemSize,n*=t.itemSize;for(let r=0,s=this.itemSize;r<s;r++)this.array[e+r]=t.array[n+r];return this}copyArray(e){return this.array.set(e),this}applyMatrix3(e){if(this.itemSize===2)for(let t=0,n=this.count;t<n;t++)Yi.fromBufferAttribute(this,t),Yi.applyMatrix3(e),this.setXY(t,Yi.x,Yi.y);else if(this.itemSize===3)for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.applyMatrix3(e),this.setXYZ(t,at.x,at.y,at.z);return this}applyMatrix4(e){for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.applyMatrix4(e),this.setXYZ(t,at.x,at.y,at.z);return this}applyNormalMatrix(e){for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.applyNormalMatrix(e),this.setXYZ(t,at.x,at.y,at.z);return this}transformDirection(e){for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.transformDirection(e),this.setXYZ(t,at.x,at.y,at.z);return this}set(e,t=0){return this.array.set(e,t),this}getComponent(e,t){let n=this.array[e*this.itemSize+t];return this.normalized&&(n=yi(n,this.array)),n}setComponent(e,t,n){return this.normalized&&(n=xt(n,this.array)),this.array[e*this.itemSize+t]=n,this}getX(e){let t=this.array[e*this.itemSize];return this.normalized&&(t=yi(t,this.array)),t}setX(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize]=t,this}getY(e){let t=this.array[e*this.itemSize+1];return this.normalized&&(t=yi(t,this.array)),t}setY(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize+1]=t,this}getZ(e){let t=this.array[e*this.itemSize+2];return this.normalized&&(t=yi(t,this.array)),t}setZ(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize+2]=t,this}getW(e){let t=this.array[e*this.itemSize+3];return this.normalized&&(t=yi(t,this.array)),t}setW(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize+3]=t,this}setXY(e,t,n){return e*=this.itemSize,this.normalized&&(t=xt(t,this.array),n=xt(n,this.array)),this.array[e+0]=t,this.array[e+1]=n,this}setXYZ(e,t,n,r){return e*=this.itemSize,this.normalized&&(t=xt(t,this.array),n=xt(n,this.array),r=xt(r,this.array)),this.array[e+0]=t,this.array[e+1]=n,this.array[e+2]=r,this}setXYZW(e,t,n,r,s){return e*=this.itemSize,this.normalized&&(t=xt(t,this.array),n=xt(n,this.array),r=xt(r,this.array),s=xt(s,this.array)),this.array[e+0]=t,this.array[e+1]=n,this.array[e+2]=r,this.array[e+3]=s,this}onUpload(e){return this.onUploadCallback=e,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const e={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(e.name=this.name),this.usage!==ul&&(e.usage=this.usage),e}}class eu extends Gt{constructor(e,t,n){super(new Uint16Array(e),t,n)}}class tu extends Gt{constructor(e,t,n){super(new Uint32Array(e),t,n)}}class ft extends Gt{constructor(e,t,n){super(new Float32Array(e),t,n)}}let vf=0;const Ct=new Je,Ys=new dt,Jn=new z,wt=new Di,Ei=new Di,ut=new z;class Lt extends Bn{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:vf++}),this.uuid=Ui(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(e){return Array.isArray(e)?this.index=new(Yc(e)?tu:eu)(e,1):this.index=e,this}getAttribute(e){return this.attributes[e]}setAttribute(e,t){return this.attributes[e]=t,this}deleteAttribute(e){return delete this.attributes[e],this}hasAttribute(e){return this.attributes[e]!==void 0}addGroup(e,t,n=0){this.groups.push({start:e,count:t,materialIndex:n})}clearGroups(){this.groups=[]}setDrawRange(e,t){this.drawRange.start=e,this.drawRange.count=t}applyMatrix4(e){const t=this.attributes.position;t!==void 0&&(t.applyMatrix4(e),t.needsUpdate=!0);const n=this.attributes.normal;if(n!==void 0){const s=new ze().getNormalMatrix(e);n.applyNormalMatrix(s),n.needsUpdate=!0}const r=this.attributes.tangent;return r!==void 0&&(r.transformDirection(e),r.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(e){return Ct.makeRotationFromQuaternion(e),this.applyMatrix4(Ct),this}rotateX(e){return Ct.makeRotationX(e),this.applyMatrix4(Ct),this}rotateY(e){return Ct.makeRotationY(e),this.applyMatrix4(Ct),this}rotateZ(e){return Ct.makeRotationZ(e),this.applyMatrix4(Ct),this}translate(e,t,n){return Ct.makeTranslation(e,t,n),this.applyMatrix4(Ct),this}scale(e,t,n){return Ct.makeScale(e,t,n),this.applyMatrix4(Ct),this}lookAt(e){return Ys.lookAt(e),Ys.updateMatrix(),this.applyMatrix4(Ys.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Jn).negate(),this.translate(Jn.x,Jn.y,Jn.z),this}setFromPoints(e){const t=[];for(let n=0,r=e.length;n<r;n++){const s=e[n];t.push(s.x,s.y,s.z||0)}return this.setAttribute("position",new ft(t,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new Di);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new z(-1/0,-1/0,-1/0),new z(1/0,1/0,1/0));return}if(e!==void 0){if(this.boundingBox.setFromBufferAttribute(e),t)for(let n=0,r=t.length;n<r;n++){const s=t[n];wt.setFromBufferAttribute(s),this.morphTargetsRelative?(ut.addVectors(this.boundingBox.min,wt.min),this.boundingBox.expandByPoint(ut),ut.addVectors(this.boundingBox.max,wt.max),this.boundingBox.expandByPoint(ut)):(this.boundingBox.expandByPoint(wt.min),this.boundingBox.expandByPoint(wt.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new Ii);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new z,1/0);return}if(e){const n=this.boundingSphere.center;if(wt.setFromBufferAttribute(e),t)for(let s=0,o=t.length;s<o;s++){const a=t[s];Ei.setFromBufferAttribute(a),this.morphTargetsRelative?(ut.addVectors(wt.min,Ei.min),wt.expandByPoint(ut),ut.addVectors(wt.max,Ei.max),wt.expandByPoint(ut)):(wt.expandByPoint(Ei.min),wt.expandByPoint(Ei.max))}wt.getCenter(n);let r=0;for(let s=0,o=e.count;s<o;s++)ut.fromBufferAttribute(e,s),r=Math.max(r,n.distanceToSquared(ut));if(t)for(let s=0,o=t.length;s<o;s++){const a=t[s],l=this.morphTargetsRelative;for(let c=0,u=a.count;c<u;c++)ut.fromBufferAttribute(a,c),l&&(Jn.fromBufferAttribute(e,c),ut.add(Jn)),r=Math.max(r,n.distanceToSquared(ut))}this.boundingSphere.radius=Math.sqrt(r),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const e=this.index,t=this.attributes;if(e===null||t.position===void 0||t.normal===void 0||t.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const n=t.position,r=t.normal,s=t.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new Gt(new Float32Array(4*n.count),4));const o=this.getAttribute("tangent"),a=[],l=[];for(let X=0;X<n.count;X++)a[X]=new z,l[X]=new z;const c=new z,u=new z,h=new z,d=new De,f=new De,g=new De,x=new z,m=new z;function p(X,E,M){c.fromBufferAttribute(n,X),u.fromBufferAttribute(n,E),h.fromBufferAttribute(n,M),d.fromBufferAttribute(s,X),f.fromBufferAttribute(s,E),g.fromBufferAttribute(s,M),u.sub(c),h.sub(c),f.sub(d),g.sub(d);const C=1/(f.x*g.y-g.x*f.y);isFinite(C)&&(x.copy(u).multiplyScalar(g.y).addScaledVector(h,-f.y).multiplyScalar(C),m.copy(h).multiplyScalar(f.x).addScaledVector(u,-g.x).multiplyScalar(C),a[X].add(x),a[E].add(x),a[M].add(x),l[X].add(m),l[E].add(m),l[M].add(m))}let T=this.groups;T.length===0&&(T=[{start:0,count:e.count}]);for(let X=0,E=T.length;X<E;++X){const M=T[X],C=M.start,H=M.count;for(let O=C,$=C+H;O<$;O+=3)p(e.getX(O+0),e.getX(O+1),e.getX(O+2))}const S=new z,A=new z,B=new z,D=new z;function P(X){B.fromBufferAttribute(r,X),D.copy(B);const E=a[X];S.copy(E),S.sub(B.multiplyScalar(B.dot(E))).normalize(),A.crossVectors(D,E);const C=A.dot(l[X])<0?-1:1;o.setXYZW(X,S.x,S.y,S.z,C)}for(let X=0,E=T.length;X<E;++X){const M=T[X],C=M.start,H=M.count;for(let O=C,$=C+H;O<$;O+=3)P(e.getX(O+0)),P(e.getX(O+1)),P(e.getX(O+2))}}computeVertexNormals(){const e=this.index,t=this.getAttribute("position");if(t!==void 0){let n=this.getAttribute("normal");if(n===void 0)n=new Gt(new Float32Array(t.count*3),3),this.setAttribute("normal",n);else for(let d=0,f=n.count;d<f;d++)n.setXYZ(d,0,0,0);const r=new z,s=new z,o=new z,a=new z,l=new z,c=new z,u=new z,h=new z;if(e)for(let d=0,f=e.count;d<f;d+=3){const g=e.getX(d+0),x=e.getX(d+1),m=e.getX(d+2);r.fromBufferAttribute(t,g),s.fromBufferAttribute(t,x),o.fromBufferAttribute(t,m),u.subVectors(o,s),h.subVectors(r,s),u.cross(h),a.fromBufferAttribute(n,g),l.fromBufferAttribute(n,x),c.fromBufferAttribute(n,m),a.add(u),l.add(u),c.add(u),n.setXYZ(g,a.x,a.y,a.z),n.setXYZ(x,l.x,l.y,l.z),n.setXYZ(m,c.x,c.y,c.z)}else for(let d=0,f=t.count;d<f;d+=3)r.fromBufferAttribute(t,d+0),s.fromBufferAttribute(t,d+1),o.fromBufferAttribute(t,d+2),u.subVectors(o,s),h.subVectors(r,s),u.cross(h),n.setXYZ(d+0,u.x,u.y,u.z),n.setXYZ(d+1,u.x,u.y,u.z),n.setXYZ(d+2,u.x,u.y,u.z);this.normalizeNormals(),n.needsUpdate=!0}}normalizeNormals(){const e=this.attributes.normal;for(let t=0,n=e.count;t<n;t++)ut.fromBufferAttribute(e,t),ut.normalize(),e.setXYZ(t,ut.x,ut.y,ut.z)}toNonIndexed(){function e(a,l){const c=a.array,u=a.itemSize,h=a.normalized,d=new c.constructor(l.length*u);let f=0,g=0;for(let x=0,m=l.length;x<m;x++){a.isInterleavedBufferAttribute?f=l[x]*a.data.stride+a.offset:f=l[x]*u;for(let p=0;p<u;p++)d[g++]=c[f++]}return new Gt(d,u,h)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const t=new Lt,n=this.index.array,r=this.attributes;for(const a in r){const l=r[a],c=e(l,n);t.setAttribute(a,c)}const s=this.morphAttributes;for(const a in s){const l=[],c=s[a];for(let u=0,h=c.length;u<h;u++){const d=c[u],f=e(d,n);l.push(f)}t.morphAttributes[a]=l}t.morphTargetsRelative=this.morphTargetsRelative;const o=this.groups;for(let a=0,l=o.length;a<l;a++){const c=o[a];t.addGroup(c.start,c.count,c.materialIndex)}return t}toJSON(){const e={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(e.uuid=this.uuid,e.type=this.type,this.name!==""&&(e.name=this.name),Object.keys(this.userData).length>0&&(e.userData=this.userData),this.parameters!==void 0){const l=this.parameters;for(const c in l)l[c]!==void 0&&(e[c]=l[c]);return e}e.data={attributes:{}};const t=this.index;t!==null&&(e.data.index={type:t.array.constructor.name,array:Array.prototype.slice.call(t.array)});const n=this.attributes;for(const l in n){const c=n[l];e.data.attributes[l]=c.toJSON(e.data)}const r={};let s=!1;for(const l in this.morphAttributes){const c=this.morphAttributes[l],u=[];for(let h=0,d=c.length;h<d;h++){const f=c[h];u.push(f.toJSON(e.data))}u.length>0&&(r[l]=u,s=!0)}s&&(e.data.morphAttributes=r,e.data.morphTargetsRelative=this.morphTargetsRelative);const o=this.groups;o.length>0&&(e.data.groups=JSON.parse(JSON.stringify(o)));const a=this.boundingSphere;return a!==null&&(e.data.boundingSphere={center:a.center.toArray(),radius:a.radius}),e}clone(){return new this.constructor().copy(this)}copy(e){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const t={};this.name=e.name;const n=e.index;n!==null&&this.setIndex(n.clone(t));const r=e.attributes;for(const c in r){const u=r[c];this.setAttribute(c,u.clone(t))}const s=e.morphAttributes;for(const c in s){const u=[],h=s[c];for(let d=0,f=h.length;d<f;d++)u.push(h[d].clone(t));this.morphAttributes[c]=u}this.morphTargetsRelative=e.morphTargetsRelative;const o=e.groups;for(let c=0,u=o.length;c<u;c++){const h=o[c];this.addGroup(h.start,h.count,h.materialIndex)}const a=e.boundingBox;a!==null&&(this.boundingBox=a.clone());const l=e.boundingSphere;return l!==null&&(this.boundingSphere=l.clone()),this.drawRange.start=e.drawRange.start,this.drawRange.count=e.drawRange.count,this.userData=e.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const El=new Je,En=new Ar,ji=new Ii,Tl=new z,Qn=new z,ei=new z,ti=new z,js=new z,$i=new z,Ki=new De,Zi=new De,Ji=new De,wl=new z,Al=new z,Cl=new z,Qi=new z,er=new z;class Ft extends dt{constructor(e=new Lt,t=new Qc){super(),this.isMesh=!0,this.type="Mesh",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),e.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=e.morphTargetInfluences.slice()),e.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},e.morphTargetDictionary)),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}updateMorphTargets(){const t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){const r=t[n[0]];if(r!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,o=r.length;s<o;s++){const a=r[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=s}}}}getVertexPosition(e,t){const n=this.geometry,r=n.attributes.position,s=n.morphAttributes.position,o=n.morphTargetsRelative;t.fromBufferAttribute(r,e);const a=this.morphTargetInfluences;if(s&&a){$i.set(0,0,0);for(let l=0,c=s.length;l<c;l++){const u=a[l],h=s[l];u!==0&&(js.fromBufferAttribute(h,e),o?$i.addScaledVector(js,u):$i.addScaledVector(js.sub(t),u))}t.add($i)}return t}raycast(e,t){const n=this.geometry,r=this.material,s=this.matrixWorld;r!==void 0&&(n.boundingSphere===null&&n.computeBoundingSphere(),ji.copy(n.boundingSphere),ji.applyMatrix4(s),En.copy(e.ray).recast(e.near),!(ji.containsPoint(En.origin)===!1&&(En.intersectSphere(ji,Tl)===null||En.origin.distanceToSquared(Tl)>(e.far-e.near)**2))&&(El.copy(s).invert(),En.copy(e.ray).applyMatrix4(El),!(n.boundingBox!==null&&En.intersectsBox(n.boundingBox)===!1)&&this._computeIntersections(e,t,En)))}_computeIntersections(e,t,n){let r;const s=this.geometry,o=this.material,a=s.index,l=s.attributes.position,c=s.attributes.uv,u=s.attributes.uv1,h=s.attributes.normal,d=s.groups,f=s.drawRange;if(a!==null)if(Array.isArray(o))for(let g=0,x=d.length;g<x;g++){const m=d[g],p=o[m.materialIndex],T=Math.max(m.start,f.start),S=Math.min(a.count,Math.min(m.start+m.count,f.start+f.count));for(let A=T,B=S;A<B;A+=3){const D=a.getX(A),P=a.getX(A+1),X=a.getX(A+2);r=tr(this,p,e,n,c,u,h,D,P,X),r&&(r.faceIndex=Math.floor(A/3),r.face.materialIndex=m.materialIndex,t.push(r))}}else{const g=Math.max(0,f.start),x=Math.min(a.count,f.start+f.count);for(let m=g,p=x;m<p;m+=3){const T=a.getX(m),S=a.getX(m+1),A=a.getX(m+2);r=tr(this,o,e,n,c,u,h,T,S,A),r&&(r.faceIndex=Math.floor(m/3),t.push(r))}}else if(l!==void 0)if(Array.isArray(o))for(let g=0,x=d.length;g<x;g++){const m=d[g],p=o[m.materialIndex],T=Math.max(m.start,f.start),S=Math.min(l.count,Math.min(m.start+m.count,f.start+f.count));for(let A=T,B=S;A<B;A+=3){const D=A,P=A+1,X=A+2;r=tr(this,p,e,n,c,u,h,D,P,X),r&&(r.faceIndex=Math.floor(A/3),r.face.materialIndex=m.materialIndex,t.push(r))}}else{const g=Math.max(0,f.start),x=Math.min(l.count,f.start+f.count);for(let m=g,p=x;m<p;m+=3){const T=m,S=m+1,A=m+2;r=tr(this,o,e,n,c,u,h,T,S,A),r&&(r.faceIndex=Math.floor(m/3),t.push(r))}}}}function xf(i,e,t,n,r,s,o,a){let l;if(e.side===yt?l=n.intersectTriangle(o,s,r,!0,a):l=n.intersectTriangle(r,s,o,e.side===mn,a),l===null)return null;er.copy(a),er.applyMatrix4(i.matrixWorld);const c=t.ray.origin.distanceTo(er);return c<t.near||c>t.far?null:{distance:c,point:er.clone(),object:i}}function tr(i,e,t,n,r,s,o,a,l,c){i.getVertexPosition(a,Qn),i.getVertexPosition(l,ei),i.getVertexPosition(c,ti);const u=xf(i,e,t,n,Qn,ei,ti,Qi);if(u){r&&(Ki.fromBufferAttribute(r,a),Zi.fromBufferAttribute(r,l),Ji.fromBufferAttribute(r,c),u.uv=zt.getInterpolation(Qi,Qn,ei,ti,Ki,Zi,Ji,new De)),s&&(Ki.fromBufferAttribute(s,a),Zi.fromBufferAttribute(s,l),Ji.fromBufferAttribute(s,c),u.uv1=zt.getInterpolation(Qi,Qn,ei,ti,Ki,Zi,Ji,new De)),o&&(wl.fromBufferAttribute(o,a),Al.fromBufferAttribute(o,l),Cl.fromBufferAttribute(o,c),u.normal=zt.getInterpolation(Qi,Qn,ei,ti,wl,Al,Cl,new z),u.normal.dot(n.direction)>0&&u.normal.multiplyScalar(-1));const h={a,b:l,c,normal:new z,materialIndex:0};zt.getNormal(Qn,ei,ti,h.normal),u.face=h}return u}class In extends Lt{constructor(e=1,t=1,n=1,r=1,s=1,o=1){super(),this.type="BoxGeometry",this.parameters={width:e,height:t,depth:n,widthSegments:r,heightSegments:s,depthSegments:o};const a=this;r=Math.floor(r),s=Math.floor(s),o=Math.floor(o);const l=[],c=[],u=[],h=[];let d=0,f=0;g("z","y","x",-1,-1,n,t,e,o,s,0),g("z","y","x",1,-1,n,t,-e,o,s,1),g("x","z","y",1,1,e,n,t,r,o,2),g("x","z","y",1,-1,e,n,-t,r,o,3),g("x","y","z",1,-1,e,t,n,r,s,4),g("x","y","z",-1,-1,e,t,-n,r,s,5),this.setIndex(l),this.setAttribute("position",new ft(c,3)),this.setAttribute("normal",new ft(u,3)),this.setAttribute("uv",new ft(h,2));function g(x,m,p,T,S,A,B,D,P,X,E){const M=A/P,C=B/X,H=A/2,O=B/2,$=D/2,V=P+1,Y=X+1;let te=0,v=0;const w=new z;for(let U=0;U<Y;U++){const N=U*C-O;for(let G=0;G<V;G++){const K=G*M-H;w[x]=K*T,w[m]=N*S,w[p]=$,c.push(w.x,w.y,w.z),w[x]=0,w[m]=0,w[p]=D>0?1:-1,u.push(w.x,w.y,w.z),h.push(G/P),h.push(1-U/X),te+=1}}for(let U=0;U<X;U++)for(let N=0;N<P;N++){const G=d+N+V*U,K=d+N+V*(U+1),I=d+(N+1)+V*(U+1),k=d+(N+1)+V*U;l.push(G,K,k),l.push(K,I,k),v+=6}a.addGroup(f,v,E),f+=v,d+=te}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new In(e.width,e.height,e.depth,e.widthSegments,e.heightSegments,e.depthSegments)}}function mi(i){const e={};for(const t in i){e[t]={};for(const n in i[t]){const r=i[t][n];r&&(r.isColor||r.isMatrix3||r.isMatrix4||r.isVector2||r.isVector3||r.isVector4||r.isTexture||r.isQuaternion)?r.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),e[t][n]=null):e[t][n]=r.clone():Array.isArray(r)?e[t][n]=r.slice():e[t][n]=r}}return e}function _t(i){const e={};for(let t=0;t<i.length;t++){const n=mi(i[t]);for(const r in n)e[r]=n[r]}return e}function yf(i){const e=[];for(let t=0;t<i.length;t++)e.push(i[t].clone());return e}function nu(i){const e=i.getRenderTarget();return e===null?i.outputColorSpace:e.isXRRenderTarget===!0?e.texture.colorSpace:je.workingColorSpace}const Sf={clone:mi,merge:_t};var Mf=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,bf=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class _n extends kn{constructor(e){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=Mf,this.fragmentShader=bf,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,e!==void 0&&this.setValues(e)}copy(e){return super.copy(e),this.fragmentShader=e.fragmentShader,this.vertexShader=e.vertexShader,this.uniforms=mi(e.uniforms),this.uniformsGroups=yf(e.uniformsGroups),this.defines=Object.assign({},e.defines),this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.fog=e.fog,this.lights=e.lights,this.clipping=e.clipping,this.extensions=Object.assign({},e.extensions),this.glslVersion=e.glslVersion,this}toJSON(e){const t=super.toJSON(e);t.glslVersion=this.glslVersion,t.uniforms={};for(const r in this.uniforms){const o=this.uniforms[r].value;o&&o.isTexture?t.uniforms[r]={type:"t",value:o.toJSON(e).uuid}:o&&o.isColor?t.uniforms[r]={type:"c",value:o.getHex()}:o&&o.isVector2?t.uniforms[r]={type:"v2",value:o.toArray()}:o&&o.isVector3?t.uniforms[r]={type:"v3",value:o.toArray()}:o&&o.isVector4?t.uniforms[r]={type:"v4",value:o.toArray()}:o&&o.isMatrix3?t.uniforms[r]={type:"m3",value:o.toArray()}:o&&o.isMatrix4?t.uniforms[r]={type:"m4",value:o.toArray()}:t.uniforms[r]={value:o}}Object.keys(this.defines).length>0&&(t.defines=this.defines),t.vertexShader=this.vertexShader,t.fragmentShader=this.fragmentShader,t.lights=this.lights,t.clipping=this.clipping;const n={};for(const r in this.extensions)this.extensions[r]===!0&&(n[r]=!0);return Object.keys(n).length>0&&(t.extensions=n),t}}class iu extends dt{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new Je,this.projectionMatrix=new Je,this.projectionMatrixInverse=new Je,this.coordinateSystem=Jt}copy(e,t){return super.copy(e,t),this.matrixWorldInverse.copy(e.matrixWorldInverse),this.projectionMatrix.copy(e.projectionMatrix),this.projectionMatrixInverse.copy(e.projectionMatrixInverse),this.coordinateSystem=e.coordinateSystem,this}getWorldDirection(e){return super.getWorldDirection(e).negate()}updateMatrixWorld(e){super.updateMatrixWorld(e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(e,t){super.updateWorldMatrix(e,t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}const on=new z,Rl=new De,Pl=new De;class Rt extends iu{constructor(e=50,t=1,n=.1,r=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=e,this.zoom=1,this.near=n,this.far=r,this.focus=10,this.aspect=t,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.fov=e.fov,this.zoom=e.zoom,this.near=e.near,this.far=e.far,this.focus=e.focus,this.aspect=e.aspect,this.view=e.view===null?null:Object.assign({},e.view),this.filmGauge=e.filmGauge,this.filmOffset=e.filmOffset,this}setFocalLength(e){const t=.5*this.getFilmHeight()/e;this.fov=_a*2*Math.atan(t),this.updateProjectionMatrix()}getFocalLength(){const e=Math.tan(dr*.5*this.fov);return .5*this.getFilmHeight()/e}getEffectiveFOV(){return _a*2*Math.atan(Math.tan(dr*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(e,t,n){on.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),t.set(on.x,on.y).multiplyScalar(-e/on.z),on.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),n.set(on.x,on.y).multiplyScalar(-e/on.z)}getViewSize(e,t){return this.getViewBounds(e,Rl,Pl),t.subVectors(Pl,Rl)}setViewOffset(e,t,n,r,s,o){this.aspect=e/t,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=n,this.view.offsetY=r,this.view.width=s,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=this.near;let t=e*Math.tan(dr*.5*this.fov)/this.zoom,n=2*t,r=this.aspect*n,s=-.5*r;const o=this.view;if(this.view!==null&&this.view.enabled){const l=o.fullWidth,c=o.fullHeight;s+=o.offsetX*r/l,t-=o.offsetY*n/c,r*=o.width/l,n*=o.height/c}const a=this.filmOffset;a!==0&&(s+=e*a/this.getFilmWidth()),this.projectionMatrix.makePerspective(s,s+r,t,t-n,e,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.fov=this.fov,t.object.zoom=this.zoom,t.object.near=this.near,t.object.far=this.far,t.object.focus=this.focus,t.object.aspect=this.aspect,this.view!==null&&(t.object.view=Object.assign({},this.view)),t.object.filmGauge=this.filmGauge,t.object.filmOffset=this.filmOffset,t}}const ni=-90,ii=1;class Ef extends dt{constructor(e,t,n){super(),this.type="CubeCamera",this.renderTarget=n,this.coordinateSystem=null,this.activeMipmapLevel=0;const r=new Rt(ni,ii,e,t);r.layers=this.layers,this.add(r);const s=new Rt(ni,ii,e,t);s.layers=this.layers,this.add(s);const o=new Rt(ni,ii,e,t);o.layers=this.layers,this.add(o);const a=new Rt(ni,ii,e,t);a.layers=this.layers,this.add(a);const l=new Rt(ni,ii,e,t);l.layers=this.layers,this.add(l);const c=new Rt(ni,ii,e,t);c.layers=this.layers,this.add(c)}updateCoordinateSystem(){const e=this.coordinateSystem,t=this.children.concat(),[n,r,s,o,a,l]=t;for(const c of t)this.remove(c);if(e===Jt)n.up.set(0,1,0),n.lookAt(1,0,0),r.up.set(0,1,0),r.lookAt(-1,0,0),s.up.set(0,0,-1),s.lookAt(0,1,0),o.up.set(0,0,1),o.lookAt(0,-1,0),a.up.set(0,1,0),a.lookAt(0,0,1),l.up.set(0,1,0),l.lookAt(0,0,-1);else if(e===xr)n.up.set(0,-1,0),n.lookAt(-1,0,0),r.up.set(0,-1,0),r.lookAt(1,0,0),s.up.set(0,0,1),s.lookAt(0,1,0),o.up.set(0,0,-1),o.lookAt(0,-1,0),a.up.set(0,-1,0),a.lookAt(0,0,1),l.up.set(0,-1,0),l.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+e);for(const c of t)this.add(c),c.updateMatrixWorld()}update(e,t){this.parent===null&&this.updateMatrixWorld();const{renderTarget:n,activeMipmapLevel:r}=this;this.coordinateSystem!==e.coordinateSystem&&(this.coordinateSystem=e.coordinateSystem,this.updateCoordinateSystem());const[s,o,a,l,c,u]=this.children,h=e.getRenderTarget(),d=e.getActiveCubeFace(),f=e.getActiveMipmapLevel(),g=e.xr.enabled;e.xr.enabled=!1;const x=n.texture.generateMipmaps;n.texture.generateMipmaps=!1,e.setRenderTarget(n,0,r),e.render(t,s),e.setRenderTarget(n,1,r),e.render(t,o),e.setRenderTarget(n,2,r),e.render(t,a),e.setRenderTarget(n,3,r),e.render(t,l),e.setRenderTarget(n,4,r),e.render(t,c),n.texture.generateMipmaps=x,e.setRenderTarget(n,5,r),e.render(t,u),e.setRenderTarget(h,d,f),e.xr.enabled=g,n.texture.needsPMREMUpdate=!0}}class ru extends St{constructor(e,t,n,r,s,o,a,l,c,u){e=e!==void 0?e:[],t=t!==void 0?t:ui,super(e,t,n,r,s,o,a,l,c,u),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(e){this.image=e}}class Tf extends Dn{constructor(e=1,t={}){super(e,e,t),this.isWebGLCubeRenderTarget=!0;const n={width:e,height:e,depth:1},r=[n,n,n,n,n,n];this.texture=new ru(r,t.mapping,t.wrapS,t.wrapT,t.magFilter,t.minFilter,t.format,t.type,t.anisotropy,t.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=t.generateMipmaps!==void 0?t.generateMipmaps:!1,this.texture.minFilter=t.minFilter!==void 0?t.minFilter:Ot}fromEquirectangularTexture(e,t){this.texture.type=t.type,this.texture.colorSpace=t.colorSpace,this.texture.generateMipmaps=t.generateMipmaps,this.texture.minFilter=t.minFilter,this.texture.magFilter=t.magFilter;const n={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},r=new In(5,5,5),s=new _n({name:"CubemapFromEquirect",uniforms:mi(n.uniforms),vertexShader:n.vertexShader,fragmentShader:n.fragmentShader,side:yt,blending:fn});s.uniforms.tEquirect.value=t;const o=new Ft(r,s),a=t.minFilter;return t.minFilter===Un&&(t.minFilter=Ot),new Ef(1,10,this).update(e,o),t.minFilter=a,o.geometry.dispose(),o.material.dispose(),this}clear(e,t,n,r){const s=e.getRenderTarget();for(let o=0;o<6;o++)e.setRenderTarget(this,o),e.clear(t,n,r);e.setRenderTarget(s)}}const $s=new z,wf=new z,Af=new ze;class ln{constructor(e=new z(1,0,0),t=0){this.isPlane=!0,this.normal=e,this.constant=t}set(e,t){return this.normal.copy(e),this.constant=t,this}setComponents(e,t,n,r){return this.normal.set(e,t,n),this.constant=r,this}setFromNormalAndCoplanarPoint(e,t){return this.normal.copy(e),this.constant=-t.dot(this.normal),this}setFromCoplanarPoints(e,t,n){const r=$s.subVectors(n,t).cross(wf.subVectors(e,t)).normalize();return this.setFromNormalAndCoplanarPoint(r,e),this}copy(e){return this.normal.copy(e.normal),this.constant=e.constant,this}normalize(){const e=1/this.normal.length();return this.normal.multiplyScalar(e),this.constant*=e,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(e){return this.normal.dot(e)+this.constant}distanceToSphere(e){return this.distanceToPoint(e.center)-e.radius}projectPoint(e,t){return t.copy(e).addScaledVector(this.normal,-this.distanceToPoint(e))}intersectLine(e,t){const n=e.delta($s),r=this.normal.dot(n);if(r===0)return this.distanceToPoint(e.start)===0?t.copy(e.start):null;const s=-(e.start.dot(this.normal)+this.constant)/r;return s<0||s>1?null:t.copy(e.start).addScaledVector(n,s)}intersectsLine(e){const t=this.distanceToPoint(e.start),n=this.distanceToPoint(e.end);return t<0&&n>0||n<0&&t>0}intersectsBox(e){return e.intersectsPlane(this)}intersectsSphere(e){return e.intersectsPlane(this)}coplanarPoint(e){return e.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(e,t){const n=t||Af.getNormalMatrix(e),r=this.coplanarPoint($s).applyMatrix4(e),s=this.normal.applyMatrix3(n).normalize();return this.constant=-r.dot(s),this}translate(e){return this.constant-=e.dot(this.normal),this}equals(e){return e.normal.equals(this.normal)&&e.constant===this.constant}clone(){return new this.constructor().copy(this)}}const Tn=new Ii,nr=new z;class Aa{constructor(e=new ln,t=new ln,n=new ln,r=new ln,s=new ln,o=new ln){this.planes=[e,t,n,r,s,o]}set(e,t,n,r,s,o){const a=this.planes;return a[0].copy(e),a[1].copy(t),a[2].copy(n),a[3].copy(r),a[4].copy(s),a[5].copy(o),this}copy(e){const t=this.planes;for(let n=0;n<6;n++)t[n].copy(e.planes[n]);return this}setFromProjectionMatrix(e,t=Jt){const n=this.planes,r=e.elements,s=r[0],o=r[1],a=r[2],l=r[3],c=r[4],u=r[5],h=r[6],d=r[7],f=r[8],g=r[9],x=r[10],m=r[11],p=r[12],T=r[13],S=r[14],A=r[15];if(n[0].setComponents(l-s,d-c,m-f,A-p).normalize(),n[1].setComponents(l+s,d+c,m+f,A+p).normalize(),n[2].setComponents(l+o,d+u,m+g,A+T).normalize(),n[3].setComponents(l-o,d-u,m-g,A-T).normalize(),n[4].setComponents(l-a,d-h,m-x,A-S).normalize(),t===Jt)n[5].setComponents(l+a,d+h,m+x,A+S).normalize();else if(t===xr)n[5].setComponents(a,h,x,S).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+t);return this}intersectsObject(e){if(e.boundingSphere!==void 0)e.boundingSphere===null&&e.computeBoundingSphere(),Tn.copy(e.boundingSphere).applyMatrix4(e.matrixWorld);else{const t=e.geometry;t.boundingSphere===null&&t.computeBoundingSphere(),Tn.copy(t.boundingSphere).applyMatrix4(e.matrixWorld)}return this.intersectsSphere(Tn)}intersectsSprite(e){return Tn.center.set(0,0,0),Tn.radius=.7071067811865476,Tn.applyMatrix4(e.matrixWorld),this.intersectsSphere(Tn)}intersectsSphere(e){const t=this.planes,n=e.center,r=-e.radius;for(let s=0;s<6;s++)if(t[s].distanceToPoint(n)<r)return!1;return!0}intersectsBox(e){const t=this.planes;for(let n=0;n<6;n++){const r=t[n];if(nr.x=r.normal.x>0?e.max.x:e.min.x,nr.y=r.normal.y>0?e.max.y:e.min.y,nr.z=r.normal.z>0?e.max.z:e.min.z,r.distanceToPoint(nr)<0)return!1}return!0}containsPoint(e){const t=this.planes;for(let n=0;n<6;n++)if(t[n].distanceToPoint(e)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function su(){let i=null,e=!1,t=null,n=null;function r(s,o){t(s,o),n=i.requestAnimationFrame(r)}return{start:function(){e!==!0&&t!==null&&(n=i.requestAnimationFrame(r),e=!0)},stop:function(){i.cancelAnimationFrame(n),e=!1},setAnimationLoop:function(s){t=s},setContext:function(s){i=s}}}function Cf(i){const e=new WeakMap;function t(a,l){const c=a.array,u=a.usage,h=c.byteLength,d=i.createBuffer();i.bindBuffer(l,d),i.bufferData(l,c,u),a.onUploadCallback();let f;if(c instanceof Float32Array)f=i.FLOAT;else if(c instanceof Uint16Array)a.isFloat16BufferAttribute?f=i.HALF_FLOAT:f=i.UNSIGNED_SHORT;else if(c instanceof Int16Array)f=i.SHORT;else if(c instanceof Uint32Array)f=i.UNSIGNED_INT;else if(c instanceof Int32Array)f=i.INT;else if(c instanceof Int8Array)f=i.BYTE;else if(c instanceof Uint8Array)f=i.UNSIGNED_BYTE;else if(c instanceof Uint8ClampedArray)f=i.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+c);return{buffer:d,type:f,bytesPerElement:c.BYTES_PER_ELEMENT,version:a.version,size:h}}function n(a,l,c){const u=l.array,h=l._updateRange,d=l.updateRanges;if(i.bindBuffer(c,a),h.count===-1&&d.length===0&&i.bufferSubData(c,0,u),d.length!==0){for(let f=0,g=d.length;f<g;f++){const x=d[f];i.bufferSubData(c,x.start*u.BYTES_PER_ELEMENT,u,x.start,x.count)}l.clearUpdateRanges()}h.count!==-1&&(i.bufferSubData(c,h.offset*u.BYTES_PER_ELEMENT,u,h.offset,h.count),h.count=-1),l.onUploadCallback()}function r(a){return a.isInterleavedBufferAttribute&&(a=a.data),e.get(a)}function s(a){a.isInterleavedBufferAttribute&&(a=a.data);const l=e.get(a);l&&(i.deleteBuffer(l.buffer),e.delete(a))}function o(a,l){if(a.isGLBufferAttribute){const u=e.get(a);(!u||u.version<a.version)&&e.set(a,{buffer:a.buffer,type:a.type,bytesPerElement:a.elementSize,version:a.version});return}a.isInterleavedBufferAttribute&&(a=a.data);const c=e.get(a);if(c===void 0)e.set(a,t(a,l));else if(c.version<a.version){if(c.size!==a.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");n(c.buffer,a,l),c.version=a.version}}return{get:r,remove:s,update:o}}class Cr extends Lt{constructor(e=1,t=1,n=1,r=1){super(),this.type="PlaneGeometry",this.parameters={width:e,height:t,widthSegments:n,heightSegments:r};const s=e/2,o=t/2,a=Math.floor(n),l=Math.floor(r),c=a+1,u=l+1,h=e/a,d=t/l,f=[],g=[],x=[],m=[];for(let p=0;p<u;p++){const T=p*d-o;for(let S=0;S<c;S++){const A=S*h-s;g.push(A,-T,0),x.push(0,0,1),m.push(S/a),m.push(1-p/l)}}for(let p=0;p<l;p++)for(let T=0;T<a;T++){const S=T+c*p,A=T+c*(p+1),B=T+1+c*(p+1),D=T+1+c*p;f.push(S,A,D),f.push(A,B,D)}this.setIndex(f),this.setAttribute("position",new ft(g,3)),this.setAttribute("normal",new ft(x,3)),this.setAttribute("uv",new ft(m,2))}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new Cr(e.width,e.height,e.widthSegments,e.heightSegments)}}var Rf=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,Pf=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,Lf=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,Uf=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,Df=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,If=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,Nf=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT ) 
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN ) 
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,Of=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,Ff=`#ifdef USE_BATCHING
	attribute float batchId;
	uniform highp sampler2D batchingTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif
#ifdef USE_BATCHING_COLOR
	uniform sampler2D batchingColorTexture;
	vec3 getBatchingColor( const in float i ) {
		int size = textureSize( batchingColorTexture, 0 ).x;
		int j = int( i );
		int x = j % size;
		int y = j / size;
		return texelFetch( batchingColorTexture, ivec2( x, y ), 0 ).rgb;
	}
#endif`,Bf=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( batchId );
#endif`,kf=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,zf=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,Vf=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,Gf=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,Hf=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,Wf=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#ifdef ALPHA_TO_COVERAGE
		float distanceToPlane, distanceGradient;
		float clipOpacity = 1.0;
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
			distanceGradient = fwidth( distanceToPlane ) / 2.0;
			clipOpacity *= smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			if ( clipOpacity == 0.0 ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			float unionClipOpacity = 1.0;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
				distanceGradient = fwidth( distanceToPlane ) / 2.0;
				unionClipOpacity *= 1.0 - smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			}
			#pragma unroll_loop_end
			clipOpacity *= 1.0 - unionClipOpacity;
		#endif
		diffuseColor.a *= clipOpacity;
		if ( diffuseColor.a == 0.0 ) discard;
	#else
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			bool clipped = true;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
			}
			#pragma unroll_loop_end
			if ( clipped ) discard;
		#endif
	#endif
#endif`,Xf=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,qf=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,Yf=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,jf=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,$f=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,Kf=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,Zf=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif
#ifdef USE_BATCHING_COLOR
	vec3 batchingColor = getBatchingColor( batchId );
	vColor.xyz *= batchingColor.xyz;
#endif`,Jf=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
float luminance( const in vec3 rgb ) {
	const vec3 weights = vec3( 0.2126729, 0.7151522, 0.0721750 );
	return dot( weights, rgb );
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,Qf=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,ep=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,tp=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,np=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,ip=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,rp=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,sp="gl_FragColor = linearToOutputTexel( gl_FragColor );",ap=`
const mat3 LINEAR_SRGB_TO_LINEAR_DISPLAY_P3 = mat3(
	vec3( 0.8224621, 0.177538, 0.0 ),
	vec3( 0.0331941, 0.9668058, 0.0 ),
	vec3( 0.0170827, 0.0723974, 0.9105199 )
);
const mat3 LINEAR_DISPLAY_P3_TO_LINEAR_SRGB = mat3(
	vec3( 1.2249401, - 0.2249404, 0.0 ),
	vec3( - 0.0420569, 1.0420571, 0.0 ),
	vec3( - 0.0196376, - 0.0786361, 1.0982735 )
);
vec4 LinearSRGBToLinearDisplayP3( in vec4 value ) {
	return vec4( value.rgb * LINEAR_SRGB_TO_LINEAR_DISPLAY_P3, value.a );
}
vec4 LinearDisplayP3ToLinearSRGB( in vec4 value ) {
	return vec4( value.rgb * LINEAR_DISPLAY_P3_TO_LINEAR_SRGB, value.a );
}
vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}
vec4 LinearToLinear( in vec4 value ) {
	return value;
}
vec4 LinearTosRGB( in vec4 value ) {
	return sRGBTransferOETF( value );
}`,op=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, envMapRotation * vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,lp=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,cp=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,up=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,hp=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,dp=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,fp=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,pp=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,mp=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,gp=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,_p=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,vp=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,xp=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,yp=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
	if ( cutoffDistance > 0.0 ) {
		distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
	}
	return distanceFalloff;
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,Sp=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,Mp=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,bp=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,Ep=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,Tp=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,wp=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_DISPERSION
	material.dispersion = dispersion;
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,Ap=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	float dispersion;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,Cp=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,Rp=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,Pp=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,Lp=`#if defined( USE_LOGDEPTHBUF )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,Up=`#if defined( USE_LOGDEPTHBUF )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Dp=`#ifdef USE_LOGDEPTHBUF
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,Ip=`#ifdef USE_LOGDEPTHBUF
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,Np=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,Op=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,Fp=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,Bp=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,kp=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,zp=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,Vp=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,Gp=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,Hp=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,Wp=`#ifdef USE_MORPHTARGETS
	#ifndef USE_INSTANCING_MORPH
		uniform float morphTargetBaseInfluence;
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	#endif
	uniform sampler2DArray morphTargetsTexture;
	uniform ivec2 morphTargetsTextureSize;
	vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
		int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
		int y = texelIndex / morphTargetsTextureSize.x;
		int x = texelIndex - y * morphTargetsTextureSize.x;
		ivec3 morphUV = ivec3( x, y, morphTargetIndex );
		return texelFetch( morphTargetsTexture, morphUV, 0 );
	}
#endif`,Xp=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,qp=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,Yp=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,jp=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,$p=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Kp=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,Zp=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,Jp=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,Qp=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,em=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,tm=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,nm=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,im=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;
const vec3 PackFactors = vec3( 256. * 256. * 256., 256. * 256., 256. );
const vec4 UnpackFactors = UnpackDownscale / vec4( PackFactors, 1. );
const float ShiftRight8 = 1. / 256.;
vec4 packDepthToRGBA( const in float v ) {
	vec4 r = vec4( fract( v * PackFactors ), v );
	r.yzw -= r.xyz * ShiftRight8;	return r * PackUpscale;
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors );
}
vec2 packDepthToRG( in highp float v ) {
	return packDepthToRGBA( v ).yx;
}
float unpackRGToDepth( const in highp vec2 v ) {
	return unpackRGBAToDepth( vec4( v.xy, 0.0, 0.0 ) );
}
vec4 pack2HalfToRGBA( vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,rm=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,sm=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,am=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,om=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,lm=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,cm=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,um=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		return step( compare, unpackRGBAToDepth( texture2D( depths, uv ) ) );
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow (sampler2D shadow, vec2 uv, float compare ){
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		float hard_shadow = step( compare , distribution.x );
		if (hard_shadow != 1.0 ) {
			float distance = compare - distribution.x ;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return shadow;
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		float shadow = 1.0;
		vec3 lightToPosition = shadowCoord.xyz;
		
		float lightToPositionLength = length( lightToPosition );
		if ( lightToPositionLength - shadowCameraFar <= 0.0 && lightToPositionLength - shadowCameraNear >= 0.0 ) {
			float dp = ( lightToPositionLength - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );			dp += shadowBias;
			vec3 bd3D = normalize( lightToPosition );
			vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
			#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
				vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
				shadow = (
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
				) * ( 1.0 / 9.0 );
			#else
				shadow = texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
			#endif
		}
		return shadow;
	}
#endif`,hm=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,dm=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,fm=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,pm=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,mm=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,gm=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,_m=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,vm=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,xm=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,ym=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,Sm=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 OptimizedCineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color *= toneMappingExposure;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	color = clamp( color, 0.0, 1.0 );
	return color;
}
vec3 NeutralToneMapping( vec3 color ) {
	const float StartCompression = 0.8 - 0.04;
	const float Desaturation = 0.15;
	color *= toneMappingExposure;
	float x = min( color.r, min( color.g, color.b ) );
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	color -= offset;
	float peak = max( color.r, max( color.g, color.b ) );
	if ( peak < StartCompression ) return color;
	float d = 1. - StartCompression;
	float newPeak = 1. - d * d / ( peak + d - StartCompression );
	color *= newPeak / peak;
	float g = 1. - 1. / ( Desaturation * ( peak - newPeak ) + 1. );
	return mix( color, vec3( newPeak ), g );
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,Mm=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.dispersion, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,bm=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float dispersion, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec4 transmittedLight;
		vec3 transmittance;
		#ifdef USE_DISPERSION
			float halfSpread = ( ior - 1.0 ) * 0.025 * dispersion;
			vec3 iors = vec3( ior - halfSpread, ior, ior + halfSpread );
			for ( int i = 0; i < 3; i ++ ) {
				vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, iors[ i ], modelMatrix );
				vec3 refractedRayExit = position + transmissionRay;
		
				vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
				vec2 refractionCoords = ndcPos.xy / ndcPos.w;
				refractionCoords += 1.0;
				refractionCoords /= 2.0;
		
				vec4 transmissionSample = getTransmissionSample( refractionCoords, roughness, iors[ i ] );
				transmittedLight[ i ] = transmissionSample[ i ];
				transmittedLight.a += transmissionSample.a;
				transmittance[ i ] = diffuseColor[ i ] * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance )[ i ];
			}
			transmittedLight.a /= 3.0;
		
		#else
		
			vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
			vec3 refractedRayExit = position + transmissionRay;
			vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
			vec2 refractionCoords = ndcPos.xy / ndcPos.w;
			refractionCoords += 1.0;
			refractionCoords /= 2.0;
			transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
			transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		
		#endif
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,Em=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,Tm=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,wm=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,Am=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const Cm=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,Rm=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,Pm=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,Lm=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
uniform mat3 backgroundRotation;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, backgroundRotation * vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, backgroundRotation * vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,Um=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,Dm=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,Im=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,Nm=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	float fragCoordZ = 0.5 * vHighPrecisionZW[0] / vHighPrecisionZW[1] + 0.5;
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#endif
}`,Om=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,Fm=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,Bm=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,km=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,zm=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,Vm=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,Gm=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,Hm=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,Wm=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,Xm=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,qm=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,Ym=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,jm=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,$m=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( 0.0, 0.0, 0.0, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), diffuseColor.a );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,Km=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,Zm=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,Jm=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,Qm=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_DISPERSION
	uniform float dispersion;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,eg=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,tg=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,ng=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,ig=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,rg=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,sg=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,ag=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix * vec4( 0.0, 0.0, 0.0, 1.0 );
	vec2 scale;
	scale.x = length( vec3( modelMatrix[ 0 ].x, modelMatrix[ 0 ].y, modelMatrix[ 0 ].z ) );
	scale.y = length( vec3( modelMatrix[ 1 ].x, modelMatrix[ 1 ].y, modelMatrix[ 1 ].z ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,og=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,ke={alphahash_fragment:Rf,alphahash_pars_fragment:Pf,alphamap_fragment:Lf,alphamap_pars_fragment:Uf,alphatest_fragment:Df,alphatest_pars_fragment:If,aomap_fragment:Nf,aomap_pars_fragment:Of,batching_pars_vertex:Ff,batching_vertex:Bf,begin_vertex:kf,beginnormal_vertex:zf,bsdfs:Vf,iridescence_fragment:Gf,bumpmap_pars_fragment:Hf,clipping_planes_fragment:Wf,clipping_planes_pars_fragment:Xf,clipping_planes_pars_vertex:qf,clipping_planes_vertex:Yf,color_fragment:jf,color_pars_fragment:$f,color_pars_vertex:Kf,color_vertex:Zf,common:Jf,cube_uv_reflection_fragment:Qf,defaultnormal_vertex:ep,displacementmap_pars_vertex:tp,displacementmap_vertex:np,emissivemap_fragment:ip,emissivemap_pars_fragment:rp,colorspace_fragment:sp,colorspace_pars_fragment:ap,envmap_fragment:op,envmap_common_pars_fragment:lp,envmap_pars_fragment:cp,envmap_pars_vertex:up,envmap_physical_pars_fragment:Sp,envmap_vertex:hp,fog_vertex:dp,fog_pars_vertex:fp,fog_fragment:pp,fog_pars_fragment:mp,gradientmap_pars_fragment:gp,lightmap_pars_fragment:_p,lights_lambert_fragment:vp,lights_lambert_pars_fragment:xp,lights_pars_begin:yp,lights_toon_fragment:Mp,lights_toon_pars_fragment:bp,lights_phong_fragment:Ep,lights_phong_pars_fragment:Tp,lights_physical_fragment:wp,lights_physical_pars_fragment:Ap,lights_fragment_begin:Cp,lights_fragment_maps:Rp,lights_fragment_end:Pp,logdepthbuf_fragment:Lp,logdepthbuf_pars_fragment:Up,logdepthbuf_pars_vertex:Dp,logdepthbuf_vertex:Ip,map_fragment:Np,map_pars_fragment:Op,map_particle_fragment:Fp,map_particle_pars_fragment:Bp,metalnessmap_fragment:kp,metalnessmap_pars_fragment:zp,morphinstance_vertex:Vp,morphcolor_vertex:Gp,morphnormal_vertex:Hp,morphtarget_pars_vertex:Wp,morphtarget_vertex:Xp,normal_fragment_begin:qp,normal_fragment_maps:Yp,normal_pars_fragment:jp,normal_pars_vertex:$p,normal_vertex:Kp,normalmap_pars_fragment:Zp,clearcoat_normal_fragment_begin:Jp,clearcoat_normal_fragment_maps:Qp,clearcoat_pars_fragment:em,iridescence_pars_fragment:tm,opaque_fragment:nm,packing:im,premultiplied_alpha_fragment:rm,project_vertex:sm,dithering_fragment:am,dithering_pars_fragment:om,roughnessmap_fragment:lm,roughnessmap_pars_fragment:cm,shadowmap_pars_fragment:um,shadowmap_pars_vertex:hm,shadowmap_vertex:dm,shadowmask_pars_fragment:fm,skinbase_vertex:pm,skinning_pars_vertex:mm,skinning_vertex:gm,skinnormal_vertex:_m,specularmap_fragment:vm,specularmap_pars_fragment:xm,tonemapping_fragment:ym,tonemapping_pars_fragment:Sm,transmission_fragment:Mm,transmission_pars_fragment:bm,uv_pars_fragment:Em,uv_pars_vertex:Tm,uv_vertex:wm,worldpos_vertex:Am,background_vert:Cm,background_frag:Rm,backgroundCube_vert:Pm,backgroundCube_frag:Lm,cube_vert:Um,cube_frag:Dm,depth_vert:Im,depth_frag:Nm,distanceRGBA_vert:Om,distanceRGBA_frag:Fm,equirect_vert:Bm,equirect_frag:km,linedashed_vert:zm,linedashed_frag:Vm,meshbasic_vert:Gm,meshbasic_frag:Hm,meshlambert_vert:Wm,meshlambert_frag:Xm,meshmatcap_vert:qm,meshmatcap_frag:Ym,meshnormal_vert:jm,meshnormal_frag:$m,meshphong_vert:Km,meshphong_frag:Zm,meshphysical_vert:Jm,meshphysical_frag:Qm,meshtoon_vert:eg,meshtoon_frag:tg,points_vert:ng,points_frag:ig,shadow_vert:rg,shadow_frag:sg,sprite_vert:ag,sprite_frag:og},fe={common:{diffuse:{value:new He(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new ze},alphaMap:{value:null},alphaMapTransform:{value:new ze},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new ze}},envmap:{envMap:{value:null},envMapRotation:{value:new ze},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new ze}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new ze}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new ze},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new ze},normalScale:{value:new De(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new ze},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new ze}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new ze}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new ze}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new He(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new He(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new ze},alphaTest:{value:0},uvTransform:{value:new ze}},sprite:{diffuse:{value:new He(16777215)},opacity:{value:1},center:{value:new De(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new ze},alphaMap:{value:null},alphaMapTransform:{value:new ze},alphaTest:{value:0}}},kt={basic:{uniforms:_t([fe.common,fe.specularmap,fe.envmap,fe.aomap,fe.lightmap,fe.fog]),vertexShader:ke.meshbasic_vert,fragmentShader:ke.meshbasic_frag},lambert:{uniforms:_t([fe.common,fe.specularmap,fe.envmap,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.fog,fe.lights,{emissive:{value:new He(0)}}]),vertexShader:ke.meshlambert_vert,fragmentShader:ke.meshlambert_frag},phong:{uniforms:_t([fe.common,fe.specularmap,fe.envmap,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.fog,fe.lights,{emissive:{value:new He(0)},specular:{value:new He(1118481)},shininess:{value:30}}]),vertexShader:ke.meshphong_vert,fragmentShader:ke.meshphong_frag},standard:{uniforms:_t([fe.common,fe.envmap,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.roughnessmap,fe.metalnessmap,fe.fog,fe.lights,{emissive:{value:new He(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:ke.meshphysical_vert,fragmentShader:ke.meshphysical_frag},toon:{uniforms:_t([fe.common,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.gradientmap,fe.fog,fe.lights,{emissive:{value:new He(0)}}]),vertexShader:ke.meshtoon_vert,fragmentShader:ke.meshtoon_frag},matcap:{uniforms:_t([fe.common,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.fog,{matcap:{value:null}}]),vertexShader:ke.meshmatcap_vert,fragmentShader:ke.meshmatcap_frag},points:{uniforms:_t([fe.points,fe.fog]),vertexShader:ke.points_vert,fragmentShader:ke.points_frag},dashed:{uniforms:_t([fe.common,fe.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:ke.linedashed_vert,fragmentShader:ke.linedashed_frag},depth:{uniforms:_t([fe.common,fe.displacementmap]),vertexShader:ke.depth_vert,fragmentShader:ke.depth_frag},normal:{uniforms:_t([fe.common,fe.bumpmap,fe.normalmap,fe.displacementmap,{opacity:{value:1}}]),vertexShader:ke.meshnormal_vert,fragmentShader:ke.meshnormal_frag},sprite:{uniforms:_t([fe.sprite,fe.fog]),vertexShader:ke.sprite_vert,fragmentShader:ke.sprite_frag},background:{uniforms:{uvTransform:{value:new ze},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:ke.background_vert,fragmentShader:ke.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new ze}},vertexShader:ke.backgroundCube_vert,fragmentShader:ke.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:ke.cube_vert,fragmentShader:ke.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:ke.equirect_vert,fragmentShader:ke.equirect_frag},distanceRGBA:{uniforms:_t([fe.common,fe.displacementmap,{referencePosition:{value:new z},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:ke.distanceRGBA_vert,fragmentShader:ke.distanceRGBA_frag},shadow:{uniforms:_t([fe.lights,fe.fog,{color:{value:new He(0)},opacity:{value:1}}]),vertexShader:ke.shadow_vert,fragmentShader:ke.shadow_frag}};kt.physical={uniforms:_t([kt.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new ze},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new ze},clearcoatNormalScale:{value:new De(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new ze},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new ze},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new ze},sheen:{value:0},sheenColor:{value:new He(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new ze},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new ze},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new ze},transmissionSamplerSize:{value:new De},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new ze},attenuationDistance:{value:0},attenuationColor:{value:new He(0)},specularColor:{value:new He(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new ze},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new ze},anisotropyVector:{value:new De},anisotropyMap:{value:null},anisotropyMapTransform:{value:new ze}}]),vertexShader:ke.meshphysical_vert,fragmentShader:ke.meshphysical_frag};const ir={r:0,b:0,g:0},wn=new Wt,lg=new Je;function cg(i,e,t,n,r,s,o){const a=new He(0);let l=s===!0?0:1,c,u,h=null,d=0,f=null;function g(T){let S=T.isScene===!0?T.background:null;return S&&S.isTexture&&(S=(T.backgroundBlurriness>0?t:e).get(S)),S}function x(T){let S=!1;const A=g(T);A===null?p(a,l):A&&A.isColor&&(p(A,1),S=!0);const B=i.xr.getEnvironmentBlendMode();B==="additive"?n.buffers.color.setClear(0,0,0,1,o):B==="alpha-blend"&&n.buffers.color.setClear(0,0,0,0,o),(i.autoClear||S)&&(n.buffers.depth.setTest(!0),n.buffers.depth.setMask(!0),n.buffers.color.setMask(!0),i.clear(i.autoClearColor,i.autoClearDepth,i.autoClearStencil))}function m(T,S){const A=g(S);A&&(A.isCubeTexture||A.mapping===Er)?(u===void 0&&(u=new Ft(new In(1,1,1),new _n({name:"BackgroundCubeMaterial",uniforms:mi(kt.backgroundCube.uniforms),vertexShader:kt.backgroundCube.vertexShader,fragmentShader:kt.backgroundCube.fragmentShader,side:yt,depthTest:!1,depthWrite:!1,fog:!1})),u.geometry.deleteAttribute("normal"),u.geometry.deleteAttribute("uv"),u.onBeforeRender=function(B,D,P){this.matrixWorld.copyPosition(P.matrixWorld)},Object.defineProperty(u.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),r.update(u)),wn.copy(S.backgroundRotation),wn.x*=-1,wn.y*=-1,wn.z*=-1,A.isCubeTexture&&A.isRenderTargetTexture===!1&&(wn.y*=-1,wn.z*=-1),u.material.uniforms.envMap.value=A,u.material.uniforms.flipEnvMap.value=A.isCubeTexture&&A.isRenderTargetTexture===!1?-1:1,u.material.uniforms.backgroundBlurriness.value=S.backgroundBlurriness,u.material.uniforms.backgroundIntensity.value=S.backgroundIntensity,u.material.uniforms.backgroundRotation.value.setFromMatrix4(lg.makeRotationFromEuler(wn)),u.material.toneMapped=je.getTransfer(A.colorSpace)!==Ze,(h!==A||d!==A.version||f!==i.toneMapping)&&(u.material.needsUpdate=!0,h=A,d=A.version,f=i.toneMapping),u.layers.enableAll(),T.unshift(u,u.geometry,u.material,0,0,null)):A&&A.isTexture&&(c===void 0&&(c=new Ft(new Cr(2,2),new _n({name:"BackgroundMaterial",uniforms:mi(kt.background.uniforms),vertexShader:kt.background.vertexShader,fragmentShader:kt.background.fragmentShader,side:mn,depthTest:!1,depthWrite:!1,fog:!1})),c.geometry.deleteAttribute("normal"),Object.defineProperty(c.material,"map",{get:function(){return this.uniforms.t2D.value}}),r.update(c)),c.material.uniforms.t2D.value=A,c.material.uniforms.backgroundIntensity.value=S.backgroundIntensity,c.material.toneMapped=je.getTransfer(A.colorSpace)!==Ze,A.matrixAutoUpdate===!0&&A.updateMatrix(),c.material.uniforms.uvTransform.value.copy(A.matrix),(h!==A||d!==A.version||f!==i.toneMapping)&&(c.material.needsUpdate=!0,h=A,d=A.version,f=i.toneMapping),c.layers.enableAll(),T.unshift(c,c.geometry,c.material,0,0,null))}function p(T,S){T.getRGB(ir,nu(i)),n.buffers.color.setClear(ir.r,ir.g,ir.b,S,o)}return{getClearColor:function(){return a},setClearColor:function(T,S=1){a.set(T),l=S,p(a,l)},getClearAlpha:function(){return l},setClearAlpha:function(T){l=T,p(a,l)},render:x,addToRenderList:m}}function ug(i,e){const t=i.getParameter(i.MAX_VERTEX_ATTRIBS),n={},r=d(null);let s=r,o=!1;function a(M,C,H,O,$){let V=!1;const Y=h(O,H,C);s!==Y&&(s=Y,c(s.object)),V=f(M,O,H,$),V&&g(M,O,H,$),$!==null&&e.update($,i.ELEMENT_ARRAY_BUFFER),(V||o)&&(o=!1,A(M,C,H,O),$!==null&&i.bindBuffer(i.ELEMENT_ARRAY_BUFFER,e.get($).buffer))}function l(){return i.createVertexArray()}function c(M){return i.bindVertexArray(M)}function u(M){return i.deleteVertexArray(M)}function h(M,C,H){const O=H.wireframe===!0;let $=n[M.id];$===void 0&&($={},n[M.id]=$);let V=$[C.id];V===void 0&&(V={},$[C.id]=V);let Y=V[O];return Y===void 0&&(Y=d(l()),V[O]=Y),Y}function d(M){const C=[],H=[],O=[];for(let $=0;$<t;$++)C[$]=0,H[$]=0,O[$]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:C,enabledAttributes:H,attributeDivisors:O,object:M,attributes:{},index:null}}function f(M,C,H,O){const $=s.attributes,V=C.attributes;let Y=0;const te=H.getAttributes();for(const v in te)if(te[v].location>=0){const U=$[v];let N=V[v];if(N===void 0&&(v==="instanceMatrix"&&M.instanceMatrix&&(N=M.instanceMatrix),v==="instanceColor"&&M.instanceColor&&(N=M.instanceColor)),U===void 0||U.attribute!==N||N&&U.data!==N.data)return!0;Y++}return s.attributesNum!==Y||s.index!==O}function g(M,C,H,O){const $={},V=C.attributes;let Y=0;const te=H.getAttributes();for(const v in te)if(te[v].location>=0){let U=V[v];U===void 0&&(v==="instanceMatrix"&&M.instanceMatrix&&(U=M.instanceMatrix),v==="instanceColor"&&M.instanceColor&&(U=M.instanceColor));const N={};N.attribute=U,U&&U.data&&(N.data=U.data),$[v]=N,Y++}s.attributes=$,s.attributesNum=Y,s.index=O}function x(){const M=s.newAttributes;for(let C=0,H=M.length;C<H;C++)M[C]=0}function m(M){p(M,0)}function p(M,C){const H=s.newAttributes,O=s.enabledAttributes,$=s.attributeDivisors;H[M]=1,O[M]===0&&(i.enableVertexAttribArray(M),O[M]=1),$[M]!==C&&(i.vertexAttribDivisor(M,C),$[M]=C)}function T(){const M=s.newAttributes,C=s.enabledAttributes;for(let H=0,O=C.length;H<O;H++)C[H]!==M[H]&&(i.disableVertexAttribArray(H),C[H]=0)}function S(M,C,H,O,$,V,Y){Y===!0?i.vertexAttribIPointer(M,C,H,$,V):i.vertexAttribPointer(M,C,H,O,$,V)}function A(M,C,H,O){x();const $=O.attributes,V=H.getAttributes(),Y=C.defaultAttributeValues;for(const te in V){const v=V[te];if(v.location>=0){let w=$[te];if(w===void 0&&(te==="instanceMatrix"&&M.instanceMatrix&&(w=M.instanceMatrix),te==="instanceColor"&&M.instanceColor&&(w=M.instanceColor)),w!==void 0){const U=w.normalized,N=w.itemSize,G=e.get(w);if(G===void 0)continue;const K=G.buffer,I=G.type,k=G.bytesPerElement,j=I===i.INT||I===i.UNSIGNED_INT||w.gpuType===kc;if(w.isInterleavedBufferAttribute){const q=w.data,le=q.stride,ve=w.offset;if(q.isInstancedInterleavedBuffer){for(let ge=0;ge<v.locationSize;ge++)p(v.location+ge,q.meshPerAttribute);M.isInstancedMesh!==!0&&O._maxInstanceCount===void 0&&(O._maxInstanceCount=q.meshPerAttribute*q.count)}else for(let ge=0;ge<v.locationSize;ge++)m(v.location+ge);i.bindBuffer(i.ARRAY_BUFFER,K);for(let ge=0;ge<v.locationSize;ge++)S(v.location+ge,N/v.locationSize,I,U,le*k,(ve+N/v.locationSize*ge)*k,j)}else{if(w.isInstancedBufferAttribute){for(let q=0;q<v.locationSize;q++)p(v.location+q,w.meshPerAttribute);M.isInstancedMesh!==!0&&O._maxInstanceCount===void 0&&(O._maxInstanceCount=w.meshPerAttribute*w.count)}else for(let q=0;q<v.locationSize;q++)m(v.location+q);i.bindBuffer(i.ARRAY_BUFFER,K);for(let q=0;q<v.locationSize;q++)S(v.location+q,N/v.locationSize,I,U,N*k,N/v.locationSize*q*k,j)}}else if(Y!==void 0){const U=Y[te];if(U!==void 0)switch(U.length){case 2:i.vertexAttrib2fv(v.location,U);break;case 3:i.vertexAttrib3fv(v.location,U);break;case 4:i.vertexAttrib4fv(v.location,U);break;default:i.vertexAttrib1fv(v.location,U)}}}}T()}function B(){X();for(const M in n){const C=n[M];for(const H in C){const O=C[H];for(const $ in O)u(O[$].object),delete O[$];delete C[H]}delete n[M]}}function D(M){if(n[M.id]===void 0)return;const C=n[M.id];for(const H in C){const O=C[H];for(const $ in O)u(O[$].object),delete O[$];delete C[H]}delete n[M.id]}function P(M){for(const C in n){const H=n[C];if(H[M.id]===void 0)continue;const O=H[M.id];for(const $ in O)u(O[$].object),delete O[$];delete H[M.id]}}function X(){E(),o=!0,s!==r&&(s=r,c(s.object))}function E(){r.geometry=null,r.program=null,r.wireframe=!1}return{setup:a,reset:X,resetDefaultState:E,dispose:B,releaseStatesOfGeometry:D,releaseStatesOfProgram:P,initAttributes:x,enableAttribute:m,disableUnusedAttributes:T}}function hg(i,e,t){let n;function r(c){n=c}function s(c,u){i.drawArrays(n,c,u),t.update(u,n,1)}function o(c,u,h){h!==0&&(i.drawArraysInstanced(n,c,u,h),t.update(u,n,h))}function a(c,u,h){if(h===0)return;const d=e.get("WEBGL_multi_draw");if(d===null)for(let f=0;f<h;f++)this.render(c[f],u[f]);else{d.multiDrawArraysWEBGL(n,c,0,u,0,h);let f=0;for(let g=0;g<h;g++)f+=u[g];t.update(f,n,1)}}function l(c,u,h,d){if(h===0)return;const f=e.get("WEBGL_multi_draw");if(f===null)for(let g=0;g<c.length;g++)o(c[g],u[g],d[g]);else{f.multiDrawArraysInstancedWEBGL(n,c,0,u,0,d,0,h);let g=0;for(let x=0;x<h;x++)g+=u[x];for(let x=0;x<d.length;x++)t.update(g,n,d[x])}}this.setMode=r,this.render=s,this.renderInstances=o,this.renderMultiDraw=a,this.renderMultiDrawInstances=l}function dg(i,e,t,n){let r;function s(){if(r!==void 0)return r;if(e.has("EXT_texture_filter_anisotropic")===!0){const D=e.get("EXT_texture_filter_anisotropic");r=i.getParameter(D.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else r=0;return r}function o(D){return!(D!==Vt&&n.convert(D)!==i.getParameter(i.IMPLEMENTATION_COLOR_READ_FORMAT))}function a(D){const P=D===Tr&&(e.has("EXT_color_buffer_half_float")||e.has("EXT_color_buffer_float"));return!(D!==gn&&n.convert(D)!==i.getParameter(i.IMPLEMENTATION_COLOR_READ_TYPE)&&D!==hn&&!P)}function l(D){if(D==="highp"){if(i.getShaderPrecisionFormat(i.VERTEX_SHADER,i.HIGH_FLOAT).precision>0&&i.getShaderPrecisionFormat(i.FRAGMENT_SHADER,i.HIGH_FLOAT).precision>0)return"highp";D="mediump"}return D==="mediump"&&i.getShaderPrecisionFormat(i.VERTEX_SHADER,i.MEDIUM_FLOAT).precision>0&&i.getShaderPrecisionFormat(i.FRAGMENT_SHADER,i.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let c=t.precision!==void 0?t.precision:"highp";const u=l(c);u!==c&&(console.warn("THREE.WebGLRenderer:",c,"not supported, using",u,"instead."),c=u);const h=t.logarithmicDepthBuffer===!0,d=i.getParameter(i.MAX_TEXTURE_IMAGE_UNITS),f=i.getParameter(i.MAX_VERTEX_TEXTURE_IMAGE_UNITS),g=i.getParameter(i.MAX_TEXTURE_SIZE),x=i.getParameter(i.MAX_CUBE_MAP_TEXTURE_SIZE),m=i.getParameter(i.MAX_VERTEX_ATTRIBS),p=i.getParameter(i.MAX_VERTEX_UNIFORM_VECTORS),T=i.getParameter(i.MAX_VARYING_VECTORS),S=i.getParameter(i.MAX_FRAGMENT_UNIFORM_VECTORS),A=f>0,B=i.getParameter(i.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:s,getMaxPrecision:l,textureFormatReadable:o,textureTypeReadable:a,precision:c,logarithmicDepthBuffer:h,maxTextures:d,maxVertexTextures:f,maxTextureSize:g,maxCubemapSize:x,maxAttributes:m,maxVertexUniforms:p,maxVaryings:T,maxFragmentUniforms:S,vertexTextures:A,maxSamples:B}}function fg(i){const e=this;let t=null,n=0,r=!1,s=!1;const o=new ln,a=new ze,l={value:null,needsUpdate:!1};this.uniform=l,this.numPlanes=0,this.numIntersection=0,this.init=function(h,d){const f=h.length!==0||d||n!==0||r;return r=d,n=h.length,f},this.beginShadows=function(){s=!0,u(null)},this.endShadows=function(){s=!1},this.setGlobalState=function(h,d){t=u(h,d,0)},this.setState=function(h,d,f){const g=h.clippingPlanes,x=h.clipIntersection,m=h.clipShadows,p=i.get(h);if(!r||g===null||g.length===0||s&&!m)s?u(null):c();else{const T=s?0:n,S=T*4;let A=p.clippingState||null;l.value=A,A=u(g,d,S,f);for(let B=0;B!==S;++B)A[B]=t[B];p.clippingState=A,this.numIntersection=x?this.numPlanes:0,this.numPlanes+=T}};function c(){l.value!==t&&(l.value=t,l.needsUpdate=n>0),e.numPlanes=n,e.numIntersection=0}function u(h,d,f,g){const x=h!==null?h.length:0;let m=null;if(x!==0){if(m=l.value,g!==!0||m===null){const p=f+x*4,T=d.matrixWorldInverse;a.getNormalMatrix(T),(m===null||m.length<p)&&(m=new Float32Array(p));for(let S=0,A=f;S!==x;++S,A+=4)o.copy(h[S]).applyMatrix4(T,a),o.normal.toArray(m,A),m[A+3]=o.constant}l.value=m,l.needsUpdate=!0}return e.numPlanes=x,e.numIntersection=0,m}}function pg(i){let e=new WeakMap;function t(o,a){return a===fa?o.mapping=ui:a===pa&&(o.mapping=hi),o}function n(o){if(o&&o.isTexture){const a=o.mapping;if(a===fa||a===pa)if(e.has(o)){const l=e.get(o).texture;return t(l,o.mapping)}else{const l=o.image;if(l&&l.height>0){const c=new Tf(l.height);return c.fromEquirectangularTexture(i,o),e.set(o,c),o.addEventListener("dispose",r),t(c.texture,o.mapping)}else return null}}return o}function r(o){const a=o.target;a.removeEventListener("dispose",r);const l=e.get(a);l!==void 0&&(e.delete(a),l.dispose())}function s(){e=new WeakMap}return{get:n,dispose:s}}class au extends iu{constructor(e=-1,t=1,n=1,r=-1,s=.1,o=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=e,this.right=t,this.top=n,this.bottom=r,this.near=s,this.far=o,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.left=e.left,this.right=e.right,this.top=e.top,this.bottom=e.bottom,this.near=e.near,this.far=e.far,this.zoom=e.zoom,this.view=e.view===null?null:Object.assign({},e.view),this}setViewOffset(e,t,n,r,s,o){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=n,this.view.offsetY=r,this.view.width=s,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=(this.right-this.left)/(2*this.zoom),t=(this.top-this.bottom)/(2*this.zoom),n=(this.right+this.left)/2,r=(this.top+this.bottom)/2;let s=n-e,o=n+e,a=r+t,l=r-t;if(this.view!==null&&this.view.enabled){const c=(this.right-this.left)/this.view.fullWidth/this.zoom,u=(this.top-this.bottom)/this.view.fullHeight/this.zoom;s+=c*this.view.offsetX,o=s+c*this.view.width,a-=u*this.view.offsetY,l=a-u*this.view.height}this.projectionMatrix.makeOrthographic(s,o,a,l,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.zoom=this.zoom,t.object.left=this.left,t.object.right=this.right,t.object.top=this.top,t.object.bottom=this.bottom,t.object.near=this.near,t.object.far=this.far,this.view!==null&&(t.object.view=Object.assign({},this.view)),t}}const ai=4,Ll=[.125,.215,.35,.446,.526,.582],Pn=20,Ks=new au,Ul=new He;let Zs=null,Js=0,Qs=0,ea=!1;const Cn=(1+Math.sqrt(5))/2,ri=1/Cn,Dl=[new z(-Cn,ri,0),new z(Cn,ri,0),new z(-ri,0,Cn),new z(ri,0,Cn),new z(0,Cn,-ri),new z(0,Cn,ri),new z(-1,1,-1),new z(1,1,-1),new z(-1,1,1),new z(1,1,1)];class Il{constructor(e){this._renderer=e,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(e,t=0,n=.1,r=100){Zs=this._renderer.getRenderTarget(),Js=this._renderer.getActiveCubeFace(),Qs=this._renderer.getActiveMipmapLevel(),ea=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(256);const s=this._allocateTargets();return s.depthBuffer=!0,this._sceneToCubeUV(e,n,r,s),t>0&&this._blur(s,0,0,t),this._applyPMREM(s),this._cleanup(s),s}fromEquirectangular(e,t=null){return this._fromTexture(e,t)}fromCubemap(e,t=null){return this._fromTexture(e,t)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Fl(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=Ol(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(e){this._lodMax=Math.floor(Math.log2(e)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let e=0;e<this._lodPlanes.length;e++)this._lodPlanes[e].dispose()}_cleanup(e){this._renderer.setRenderTarget(Zs,Js,Qs),this._renderer.xr.enabled=ea,e.scissorTest=!1,rr(e,0,0,e.width,e.height)}_fromTexture(e,t){e.mapping===ui||e.mapping===hi?this._setSize(e.image.length===0?16:e.image[0].width||e.image[0].image.width):this._setSize(e.image.width/4),Zs=this._renderer.getRenderTarget(),Js=this._renderer.getActiveCubeFace(),Qs=this._renderer.getActiveMipmapLevel(),ea=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;const n=t||this._allocateTargets();return this._textureToCubeUV(e,n),this._applyPMREM(n),this._cleanup(n),n}_allocateTargets(){const e=3*Math.max(this._cubeSize,112),t=4*this._cubeSize,n={magFilter:Ot,minFilter:Ot,generateMipmaps:!1,type:Tr,format:Vt,colorSpace:vn,depthBuffer:!1},r=Nl(e,t,n);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==e||this._pingPongRenderTarget.height!==t){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=Nl(e,t,n);const{_lodMax:s}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=mg(s)),this._blurMaterial=gg(s,e,t)}return r}_compileMaterial(e){const t=new Ft(this._lodPlanes[0],e);this._renderer.compile(t,Ks)}_sceneToCubeUV(e,t,n,r){const a=new Rt(90,1,t,n),l=[1,-1,1,1,1,1],c=[1,1,1,-1,-1,-1],u=this._renderer,h=u.autoClear,d=u.toneMapping;u.getClearColor(Ul),u.toneMapping=pn,u.autoClear=!1;const f=new Qc({name:"PMREM.Background",side:yt,depthWrite:!1,depthTest:!1}),g=new Ft(new In,f);let x=!1;const m=e.background;m?m.isColor&&(f.color.copy(m),e.background=null,x=!0):(f.color.copy(Ul),x=!0);for(let p=0;p<6;p++){const T=p%3;T===0?(a.up.set(0,l[p],0),a.lookAt(c[p],0,0)):T===1?(a.up.set(0,0,l[p]),a.lookAt(0,c[p],0)):(a.up.set(0,l[p],0),a.lookAt(0,0,c[p]));const S=this._cubeSize;rr(r,T*S,p>2?S:0,S,S),u.setRenderTarget(r),x&&u.render(g,a),u.render(e,a)}g.geometry.dispose(),g.material.dispose(),u.toneMapping=d,u.autoClear=h,e.background=m}_textureToCubeUV(e,t){const n=this._renderer,r=e.mapping===ui||e.mapping===hi;r?(this._cubemapMaterial===null&&(this._cubemapMaterial=Fl()),this._cubemapMaterial.uniforms.flipEnvMap.value=e.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=Ol());const s=r?this._cubemapMaterial:this._equirectMaterial,o=new Ft(this._lodPlanes[0],s),a=s.uniforms;a.envMap.value=e;const l=this._cubeSize;rr(t,0,0,3*l,2*l),n.setRenderTarget(t),n.render(o,Ks)}_applyPMREM(e){const t=this._renderer,n=t.autoClear;t.autoClear=!1;const r=this._lodPlanes.length;for(let s=1;s<r;s++){const o=Math.sqrt(this._sigmas[s]*this._sigmas[s]-this._sigmas[s-1]*this._sigmas[s-1]),a=Dl[(r-s-1)%Dl.length];this._blur(e,s-1,s,o,a)}t.autoClear=n}_blur(e,t,n,r,s){const o=this._pingPongRenderTarget;this._halfBlur(e,o,t,n,r,"latitudinal",s),this._halfBlur(o,e,n,n,r,"longitudinal",s)}_halfBlur(e,t,n,r,s,o,a){const l=this._renderer,c=this._blurMaterial;o!=="latitudinal"&&o!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const u=3,h=new Ft(this._lodPlanes[r],c),d=c.uniforms,f=this._sizeLods[n]-1,g=isFinite(s)?Math.PI/(2*f):2*Math.PI/(2*Pn-1),x=s/g,m=isFinite(s)?1+Math.floor(u*x):Pn;m>Pn&&console.warn(`sigmaRadians, ${s}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${Pn}`);const p=[];let T=0;for(let P=0;P<Pn;++P){const X=P/x,E=Math.exp(-X*X/2);p.push(E),P===0?T+=E:P<m&&(T+=2*E)}for(let P=0;P<p.length;P++)p[P]=p[P]/T;d.envMap.value=e.texture,d.samples.value=m,d.weights.value=p,d.latitudinal.value=o==="latitudinal",a&&(d.poleAxis.value=a);const{_lodMax:S}=this;d.dTheta.value=g,d.mipInt.value=S-n;const A=this._sizeLods[r],B=3*A*(r>S-ai?r-S+ai:0),D=4*(this._cubeSize-A);rr(t,B,D,3*A,2*A),l.setRenderTarget(t),l.render(h,Ks)}}function mg(i){const e=[],t=[],n=[];let r=i;const s=i-ai+1+Ll.length;for(let o=0;o<s;o++){const a=Math.pow(2,r);t.push(a);let l=1/a;o>i-ai?l=Ll[o-i+ai-1]:o===0&&(l=0),n.push(l);const c=1/(a-2),u=-c,h=1+c,d=[u,u,h,u,h,h,u,u,h,h,u,h],f=6,g=6,x=3,m=2,p=1,T=new Float32Array(x*g*f),S=new Float32Array(m*g*f),A=new Float32Array(p*g*f);for(let D=0;D<f;D++){const P=D%3*2/3-1,X=D>2?0:-1,E=[P,X,0,P+2/3,X,0,P+2/3,X+1,0,P,X,0,P+2/3,X+1,0,P,X+1,0];T.set(E,x*g*D),S.set(d,m*g*D);const M=[D,D,D,D,D,D];A.set(M,p*g*D)}const B=new Lt;B.setAttribute("position",new Gt(T,x)),B.setAttribute("uv",new Gt(S,m)),B.setAttribute("faceIndex",new Gt(A,p)),e.push(B),r>ai&&r--}return{lodPlanes:e,sizeLods:t,sigmas:n}}function Nl(i,e,t){const n=new Dn(i,e,t);return n.texture.mapping=Er,n.texture.name="PMREM.cubeUv",n.scissorTest=!0,n}function rr(i,e,t,n,r){i.viewport.set(e,t,n,r),i.scissor.set(e,t,n,r)}function gg(i,e,t){const n=new Float32Array(Pn),r=new z(0,1,0);return new _n({name:"SphericalGaussianBlur",defines:{n:Pn,CUBEUV_TEXEL_WIDTH:1/e,CUBEUV_TEXEL_HEIGHT:1/t,CUBEUV_MAX_MIP:`${i}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:n},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:r}},vertexShader:Ca(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:fn,depthTest:!1,depthWrite:!1})}function Ol(){return new _n({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:Ca(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:fn,depthTest:!1,depthWrite:!1})}function Fl(){return new _n({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:Ca(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:fn,depthTest:!1,depthWrite:!1})}function Ca(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function _g(i){let e=new WeakMap,t=null;function n(a){if(a&&a.isTexture){const l=a.mapping,c=l===fa||l===pa,u=l===ui||l===hi;if(c||u){let h=e.get(a);const d=h!==void 0?h.texture.pmremVersion:0;if(a.isRenderTargetTexture&&a.pmremVersion!==d)return t===null&&(t=new Il(i)),h=c?t.fromEquirectangular(a,h):t.fromCubemap(a,h),h.texture.pmremVersion=a.pmremVersion,e.set(a,h),h.texture;if(h!==void 0)return h.texture;{const f=a.image;return c&&f&&f.height>0||u&&f&&r(f)?(t===null&&(t=new Il(i)),h=c?t.fromEquirectangular(a):t.fromCubemap(a),h.texture.pmremVersion=a.pmremVersion,e.set(a,h),a.addEventListener("dispose",s),h.texture):null}}}return a}function r(a){let l=0;const c=6;for(let u=0;u<c;u++)a[u]!==void 0&&l++;return l===c}function s(a){const l=a.target;l.removeEventListener("dispose",s);const c=e.get(l);c!==void 0&&(e.delete(l),c.dispose())}function o(){e=new WeakMap,t!==null&&(t.dispose(),t=null)}return{get:n,dispose:o}}function vg(i){const e={};function t(n){if(e[n]!==void 0)return e[n];let r;switch(n){case"WEBGL_depth_texture":r=i.getExtension("WEBGL_depth_texture")||i.getExtension("MOZ_WEBGL_depth_texture")||i.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":r=i.getExtension("EXT_texture_filter_anisotropic")||i.getExtension("MOZ_EXT_texture_filter_anisotropic")||i.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":r=i.getExtension("WEBGL_compressed_texture_s3tc")||i.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||i.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":r=i.getExtension("WEBGL_compressed_texture_pvrtc")||i.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:r=i.getExtension(n)}return e[n]=r,r}return{has:function(n){return t(n)!==null},init:function(){t("EXT_color_buffer_float"),t("WEBGL_clip_cull_distance"),t("OES_texture_float_linear"),t("EXT_color_buffer_half_float"),t("WEBGL_multisampled_render_to_texture"),t("WEBGL_render_shared_exponent")},get:function(n){const r=t(n);return r===null&&jc("THREE.WebGLRenderer: "+n+" extension not supported."),r}}}function xg(i,e,t,n){const r={},s=new WeakMap;function o(h){const d=h.target;d.index!==null&&e.remove(d.index);for(const g in d.attributes)e.remove(d.attributes[g]);for(const g in d.morphAttributes){const x=d.morphAttributes[g];for(let m=0,p=x.length;m<p;m++)e.remove(x[m])}d.removeEventListener("dispose",o),delete r[d.id];const f=s.get(d);f&&(e.remove(f),s.delete(d)),n.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,t.memory.geometries--}function a(h,d){return r[d.id]===!0||(d.addEventListener("dispose",o),r[d.id]=!0,t.memory.geometries++),d}function l(h){const d=h.attributes;for(const g in d)e.update(d[g],i.ARRAY_BUFFER);const f=h.morphAttributes;for(const g in f){const x=f[g];for(let m=0,p=x.length;m<p;m++)e.update(x[m],i.ARRAY_BUFFER)}}function c(h){const d=[],f=h.index,g=h.attributes.position;let x=0;if(f!==null){const T=f.array;x=f.version;for(let S=0,A=T.length;S<A;S+=3){const B=T[S+0],D=T[S+1],P=T[S+2];d.push(B,D,D,P,P,B)}}else if(g!==void 0){const T=g.array;x=g.version;for(let S=0,A=T.length/3-1;S<A;S+=3){const B=S+0,D=S+1,P=S+2;d.push(B,D,D,P,P,B)}}else return;const m=new(Yc(d)?tu:eu)(d,1);m.version=x;const p=s.get(h);p&&e.remove(p),s.set(h,m)}function u(h){const d=s.get(h);if(d){const f=h.index;f!==null&&d.version<f.version&&c(h)}else c(h);return s.get(h)}return{get:a,update:l,getWireframeAttribute:u}}function yg(i,e,t){let n;function r(d){n=d}let s,o;function a(d){s=d.type,o=d.bytesPerElement}function l(d,f){i.drawElements(n,f,s,d*o),t.update(f,n,1)}function c(d,f,g){g!==0&&(i.drawElementsInstanced(n,f,s,d*o,g),t.update(f,n,g))}function u(d,f,g){if(g===0)return;const x=e.get("WEBGL_multi_draw");if(x===null)for(let m=0;m<g;m++)this.render(d[m]/o,f[m]);else{x.multiDrawElementsWEBGL(n,f,0,s,d,0,g);let m=0;for(let p=0;p<g;p++)m+=f[p];t.update(m,n,1)}}function h(d,f,g,x){if(g===0)return;const m=e.get("WEBGL_multi_draw");if(m===null)for(let p=0;p<d.length;p++)c(d[p]/o,f[p],x[p]);else{m.multiDrawElementsInstancedWEBGL(n,f,0,s,d,0,x,0,g);let p=0;for(let T=0;T<g;T++)p+=f[T];for(let T=0;T<x.length;T++)t.update(p,n,x[T])}}this.setMode=r,this.setIndex=a,this.render=l,this.renderInstances=c,this.renderMultiDraw=u,this.renderMultiDrawInstances=h}function Sg(i){const e={geometries:0,textures:0},t={frame:0,calls:0,triangles:0,points:0,lines:0};function n(s,o,a){switch(t.calls++,o){case i.TRIANGLES:t.triangles+=a*(s/3);break;case i.LINES:t.lines+=a*(s/2);break;case i.LINE_STRIP:t.lines+=a*(s-1);break;case i.LINE_LOOP:t.lines+=a*s;break;case i.POINTS:t.points+=a*s;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",o);break}}function r(){t.calls=0,t.triangles=0,t.points=0,t.lines=0}return{memory:e,render:t,programs:null,autoReset:!0,reset:r,update:n}}function Mg(i,e,t){const n=new WeakMap,r=new ht;function s(o,a,l){const c=o.morphTargetInfluences,u=a.morphAttributes.position||a.morphAttributes.normal||a.morphAttributes.color,h=u!==void 0?u.length:0;let d=n.get(a);if(d===void 0||d.count!==h){let M=function(){X.dispose(),n.delete(a),a.removeEventListener("dispose",M)};var f=M;d!==void 0&&d.texture.dispose();const g=a.morphAttributes.position!==void 0,x=a.morphAttributes.normal!==void 0,m=a.morphAttributes.color!==void 0,p=a.morphAttributes.position||[],T=a.morphAttributes.normal||[],S=a.morphAttributes.color||[];let A=0;g===!0&&(A=1),x===!0&&(A=2),m===!0&&(A=3);let B=a.attributes.position.count*A,D=1;B>e.maxTextureSize&&(D=Math.ceil(B/e.maxTextureSize),B=e.maxTextureSize);const P=new Float32Array(B*D*4*h),X=new Kc(P,B,D,h);X.type=hn,X.needsUpdate=!0;const E=A*4;for(let C=0;C<h;C++){const H=p[C],O=T[C],$=S[C],V=B*D*4*C;for(let Y=0;Y<H.count;Y++){const te=Y*E;g===!0&&(r.fromBufferAttribute(H,Y),P[V+te+0]=r.x,P[V+te+1]=r.y,P[V+te+2]=r.z,P[V+te+3]=0),x===!0&&(r.fromBufferAttribute(O,Y),P[V+te+4]=r.x,P[V+te+5]=r.y,P[V+te+6]=r.z,P[V+te+7]=0),m===!0&&(r.fromBufferAttribute($,Y),P[V+te+8]=r.x,P[V+te+9]=r.y,P[V+te+10]=r.z,P[V+te+11]=$.itemSize===4?r.w:1)}}d={count:h,texture:X,size:new De(B,D)},n.set(a,d),a.addEventListener("dispose",M)}if(o.isInstancedMesh===!0&&o.morphTexture!==null)l.getUniforms().setValue(i,"morphTexture",o.morphTexture,t);else{let g=0;for(let m=0;m<c.length;m++)g+=c[m];const x=a.morphTargetsRelative?1:1-g;l.getUniforms().setValue(i,"morphTargetBaseInfluence",x),l.getUniforms().setValue(i,"morphTargetInfluences",c)}l.getUniforms().setValue(i,"morphTargetsTexture",d.texture,t),l.getUniforms().setValue(i,"morphTargetsTextureSize",d.size)}return{update:s}}function bg(i,e,t,n){let r=new WeakMap;function s(l){const c=n.render.frame,u=l.geometry,h=e.get(l,u);if(r.get(h)!==c&&(e.update(h),r.set(h,c)),l.isInstancedMesh&&(l.hasEventListener("dispose",a)===!1&&l.addEventListener("dispose",a),r.get(l)!==c&&(t.update(l.instanceMatrix,i.ARRAY_BUFFER),l.instanceColor!==null&&t.update(l.instanceColor,i.ARRAY_BUFFER),r.set(l,c))),l.isSkinnedMesh){const d=l.skeleton;r.get(d)!==c&&(d.update(),r.set(d,c))}return h}function o(){r=new WeakMap}function a(l){const c=l.target;c.removeEventListener("dispose",a),t.remove(c.instanceMatrix),c.instanceColor!==null&&t.remove(c.instanceColor)}return{update:s,dispose:o}}class ou extends St{constructor(e,t,n,r,s,o,a,l,c,u=li){if(u!==li&&u!==pi)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");n===void 0&&u===li&&(n=di),n===void 0&&u===pi&&(n=fi),super(null,r,s,o,a,l,u,n,c),this.isDepthTexture=!0,this.image={width:e,height:t},this.magFilter=a!==void 0?a:Pt,this.minFilter=l!==void 0?l:Pt,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(e){return super.copy(e),this.compareFunction=e.compareFunction,this}toJSON(e){const t=super.toJSON(e);return this.compareFunction!==null&&(t.compareFunction=this.compareFunction),t}}const lu=new St,cu=new ou(1,1);cu.compareFunction=qc;const uu=new Kc,hu=new cf,du=new ru,Bl=[],kl=[],zl=new Float32Array(16),Vl=new Float32Array(9),Gl=new Float32Array(4);function vi(i,e,t){const n=i[0];if(n<=0||n>0)return i;const r=e*t;let s=Bl[r];if(s===void 0&&(s=new Float32Array(r),Bl[r]=s),e!==0){n.toArray(s,0);for(let o=1,a=0;o!==e;++o)a+=t,i[o].toArray(s,a)}return s}function ot(i,e){if(i.length!==e.length)return!1;for(let t=0,n=i.length;t<n;t++)if(i[t]!==e[t])return!1;return!0}function lt(i,e){for(let t=0,n=e.length;t<n;t++)i[t]=e[t]}function Rr(i,e){let t=kl[e];t===void 0&&(t=new Int32Array(e),kl[e]=t);for(let n=0;n!==e;++n)t[n]=i.allocateTextureUnit();return t}function Eg(i,e){const t=this.cache;t[0]!==e&&(i.uniform1f(this.addr,e),t[0]=e)}function Tg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(i.uniform2f(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(ot(t,e))return;i.uniform2fv(this.addr,e),lt(t,e)}}function wg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(i.uniform3f(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else if(e.r!==void 0)(t[0]!==e.r||t[1]!==e.g||t[2]!==e.b)&&(i.uniform3f(this.addr,e.r,e.g,e.b),t[0]=e.r,t[1]=e.g,t[2]=e.b);else{if(ot(t,e))return;i.uniform3fv(this.addr,e),lt(t,e)}}function Ag(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(i.uniform4f(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(ot(t,e))return;i.uniform4fv(this.addr,e),lt(t,e)}}function Cg(i,e){const t=this.cache,n=e.elements;if(n===void 0){if(ot(t,e))return;i.uniformMatrix2fv(this.addr,!1,e),lt(t,e)}else{if(ot(t,n))return;Gl.set(n),i.uniformMatrix2fv(this.addr,!1,Gl),lt(t,n)}}function Rg(i,e){const t=this.cache,n=e.elements;if(n===void 0){if(ot(t,e))return;i.uniformMatrix3fv(this.addr,!1,e),lt(t,e)}else{if(ot(t,n))return;Vl.set(n),i.uniformMatrix3fv(this.addr,!1,Vl),lt(t,n)}}function Pg(i,e){const t=this.cache,n=e.elements;if(n===void 0){if(ot(t,e))return;i.uniformMatrix4fv(this.addr,!1,e),lt(t,e)}else{if(ot(t,n))return;zl.set(n),i.uniformMatrix4fv(this.addr,!1,zl),lt(t,n)}}function Lg(i,e){const t=this.cache;t[0]!==e&&(i.uniform1i(this.addr,e),t[0]=e)}function Ug(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(i.uniform2i(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(ot(t,e))return;i.uniform2iv(this.addr,e),lt(t,e)}}function Dg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(i.uniform3i(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(ot(t,e))return;i.uniform3iv(this.addr,e),lt(t,e)}}function Ig(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(i.uniform4i(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(ot(t,e))return;i.uniform4iv(this.addr,e),lt(t,e)}}function Ng(i,e){const t=this.cache;t[0]!==e&&(i.uniform1ui(this.addr,e),t[0]=e)}function Og(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(i.uniform2ui(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(ot(t,e))return;i.uniform2uiv(this.addr,e),lt(t,e)}}function Fg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(i.uniform3ui(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(ot(t,e))return;i.uniform3uiv(this.addr,e),lt(t,e)}}function Bg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(i.uniform4ui(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(ot(t,e))return;i.uniform4uiv(this.addr,e),lt(t,e)}}function kg(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r);const s=this.type===i.SAMPLER_2D_SHADOW?cu:lu;t.setTexture2D(e||s,r)}function zg(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r),t.setTexture3D(e||hu,r)}function Vg(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r),t.setTextureCube(e||du,r)}function Gg(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r),t.setTexture2DArray(e||uu,r)}function Hg(i){switch(i){case 5126:return Eg;case 35664:return Tg;case 35665:return wg;case 35666:return Ag;case 35674:return Cg;case 35675:return Rg;case 35676:return Pg;case 5124:case 35670:return Lg;case 35667:case 35671:return Ug;case 35668:case 35672:return Dg;case 35669:case 35673:return Ig;case 5125:return Ng;case 36294:return Og;case 36295:return Fg;case 36296:return Bg;case 35678:case 36198:case 36298:case 36306:case 35682:return kg;case 35679:case 36299:case 36307:return zg;case 35680:case 36300:case 36308:case 36293:return Vg;case 36289:case 36303:case 36311:case 36292:return Gg}}function Wg(i,e){i.uniform1fv(this.addr,e)}function Xg(i,e){const t=vi(e,this.size,2);i.uniform2fv(this.addr,t)}function qg(i,e){const t=vi(e,this.size,3);i.uniform3fv(this.addr,t)}function Yg(i,e){const t=vi(e,this.size,4);i.uniform4fv(this.addr,t)}function jg(i,e){const t=vi(e,this.size,4);i.uniformMatrix2fv(this.addr,!1,t)}function $g(i,e){const t=vi(e,this.size,9);i.uniformMatrix3fv(this.addr,!1,t)}function Kg(i,e){const t=vi(e,this.size,16);i.uniformMatrix4fv(this.addr,!1,t)}function Zg(i,e){i.uniform1iv(this.addr,e)}function Jg(i,e){i.uniform2iv(this.addr,e)}function Qg(i,e){i.uniform3iv(this.addr,e)}function e_(i,e){i.uniform4iv(this.addr,e)}function t_(i,e){i.uniform1uiv(this.addr,e)}function n_(i,e){i.uniform2uiv(this.addr,e)}function i_(i,e){i.uniform3uiv(this.addr,e)}function r_(i,e){i.uniform4uiv(this.addr,e)}function s_(i,e,t){const n=this.cache,r=e.length,s=Rr(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTexture2D(e[o]||lu,s[o])}function a_(i,e,t){const n=this.cache,r=e.length,s=Rr(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTexture3D(e[o]||hu,s[o])}function o_(i,e,t){const n=this.cache,r=e.length,s=Rr(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTextureCube(e[o]||du,s[o])}function l_(i,e,t){const n=this.cache,r=e.length,s=Rr(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTexture2DArray(e[o]||uu,s[o])}function c_(i){switch(i){case 5126:return Wg;case 35664:return Xg;case 35665:return qg;case 35666:return Yg;case 35674:return jg;case 35675:return $g;case 35676:return Kg;case 5124:case 35670:return Zg;case 35667:case 35671:return Jg;case 35668:case 35672:return Qg;case 35669:case 35673:return e_;case 5125:return t_;case 36294:return n_;case 36295:return i_;case 36296:return r_;case 35678:case 36198:case 36298:case 36306:case 35682:return s_;case 35679:case 36299:case 36307:return a_;case 35680:case 36300:case 36308:case 36293:return o_;case 36289:case 36303:case 36311:case 36292:return l_}}class u_{constructor(e,t,n){this.id=e,this.addr=n,this.cache=[],this.type=t.type,this.setValue=Hg(t.type)}}class h_{constructor(e,t,n){this.id=e,this.addr=n,this.cache=[],this.type=t.type,this.size=t.size,this.setValue=c_(t.type)}}class d_{constructor(e){this.id=e,this.seq=[],this.map={}}setValue(e,t,n){const r=this.seq;for(let s=0,o=r.length;s!==o;++s){const a=r[s];a.setValue(e,t[a.id],n)}}}const ta=/(\w+)(\])?(\[|\.)?/g;function Hl(i,e){i.seq.push(e),i.map[e.id]=e}function f_(i,e,t){const n=i.name,r=n.length;for(ta.lastIndex=0;;){const s=ta.exec(n),o=ta.lastIndex;let a=s[1];const l=s[2]==="]",c=s[3];if(l&&(a=a|0),c===void 0||c==="["&&o+2===r){Hl(t,c===void 0?new u_(a,i,e):new h_(a,i,e));break}else{let h=t.map[a];h===void 0&&(h=new d_(a),Hl(t,h)),t=h}}}class fr{constructor(e,t){this.seq=[],this.map={};const n=e.getProgramParameter(t,e.ACTIVE_UNIFORMS);for(let r=0;r<n;++r){const s=e.getActiveUniform(t,r),o=e.getUniformLocation(t,s.name);f_(s,o,this)}}setValue(e,t,n,r){const s=this.map[t];s!==void 0&&s.setValue(e,n,r)}setOptional(e,t,n){const r=t[n];r!==void 0&&this.setValue(e,n,r)}static upload(e,t,n,r){for(let s=0,o=t.length;s!==o;++s){const a=t[s],l=n[a.id];l.needsUpdate!==!1&&a.setValue(e,l.value,r)}}static seqWithValue(e,t){const n=[];for(let r=0,s=e.length;r!==s;++r){const o=e[r];o.id in t&&n.push(o)}return n}}function Wl(i,e,t){const n=i.createShader(e);return i.shaderSource(n,t),i.compileShader(n),n}const p_=37297;let m_=0;function g_(i,e){const t=i.split(`
`),n=[],r=Math.max(e-6,0),s=Math.min(e+6,t.length);for(let o=r;o<s;o++){const a=o+1;n.push(`${a===e?">":" "} ${a}: ${t[o]}`)}return n.join(`
`)}function __(i){const e=je.getPrimaries(je.workingColorSpace),t=je.getPrimaries(i);let n;switch(e===t?n="":e===vr&&t===_r?n="LinearDisplayP3ToLinearSRGB":e===_r&&t===vr&&(n="LinearSRGBToLinearDisplayP3"),i){case vn:case wr:return[n,"LinearTransferOETF"];case Bt:case wa:return[n,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",i),[n,"LinearTransferOETF"]}}function Xl(i,e,t){const n=i.getShaderParameter(e,i.COMPILE_STATUS),r=i.getShaderInfoLog(e).trim();if(n&&r==="")return"";const s=/ERROR: 0:(\d+)/.exec(r);if(s){const o=parseInt(s[1]);return t.toUpperCase()+`

`+r+`

`+g_(i.getShaderSource(e),o)}else return r}function v_(i,e){const t=__(e);return`vec4 ${i}( vec4 value ) { return ${t[0]}( ${t[1]}( value ) ); }`}function x_(i,e){let t;switch(e){case Td:t="Linear";break;case wd:t="Reinhard";break;case Ad:t="OptimizedCineon";break;case Cd:t="ACESFilmic";break;case Pd:t="AgX";break;case Ld:t="Neutral";break;case Rd:t="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",e),t="Linear"}return"vec3 "+i+"( vec3 color ) { return "+t+"ToneMapping( color ); }"}function y_(i){return[i.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",i.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(wi).join(`
`)}function S_(i){const e=[];for(const t in i){const n=i[t];n!==!1&&e.push("#define "+t+" "+n)}return e.join(`
`)}function M_(i,e){const t={},n=i.getProgramParameter(e,i.ACTIVE_ATTRIBUTES);for(let r=0;r<n;r++){const s=i.getActiveAttrib(e,r),o=s.name;let a=1;s.type===i.FLOAT_MAT2&&(a=2),s.type===i.FLOAT_MAT3&&(a=3),s.type===i.FLOAT_MAT4&&(a=4),t[o]={type:s.type,location:i.getAttribLocation(e,o),locationSize:a}}return t}function wi(i){return i!==""}function ql(i,e){const t=e.numSpotLightShadows+e.numSpotLightMaps-e.numSpotLightShadowsWithMaps;return i.replace(/NUM_DIR_LIGHTS/g,e.numDirLights).replace(/NUM_SPOT_LIGHTS/g,e.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,e.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,t).replace(/NUM_RECT_AREA_LIGHTS/g,e.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,e.numPointLights).replace(/NUM_HEMI_LIGHTS/g,e.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,e.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,e.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,e.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,e.numPointLightShadows)}function Yl(i,e){return i.replace(/NUM_CLIPPING_PLANES/g,e.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,e.numClippingPlanes-e.numClipIntersection)}const b_=/^[ \t]*#include +<([\w\d./]+)>/gm;function va(i){return i.replace(b_,T_)}const E_=new Map;function T_(i,e){let t=ke[e];if(t===void 0){const n=E_.get(e);if(n!==void 0)t=ke[n],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',e,n);else throw new Error("Can not resolve #include <"+e+">")}return va(t)}const w_=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function jl(i){return i.replace(w_,A_)}function A_(i,e,t,n){let r="";for(let s=parseInt(e);s<parseInt(t);s++)r+=n.replace(/\[\s*i\s*\]/g,"[ "+s+" ]").replace(/UNROLLED_LOOP_INDEX/g,s);return r}function $l(i){let e=`precision ${i.precision} float;
	precision ${i.precision} int;
	precision ${i.precision} sampler2D;
	precision ${i.precision} samplerCube;
	precision ${i.precision} sampler3D;
	precision ${i.precision} sampler2DArray;
	precision ${i.precision} sampler2DShadow;
	precision ${i.precision} samplerCubeShadow;
	precision ${i.precision} sampler2DArrayShadow;
	precision ${i.precision} isampler2D;
	precision ${i.precision} isampler3D;
	precision ${i.precision} isamplerCube;
	precision ${i.precision} isampler2DArray;
	precision ${i.precision} usampler2D;
	precision ${i.precision} usampler3D;
	precision ${i.precision} usamplerCube;
	precision ${i.precision} usampler2DArray;
	`;return i.precision==="highp"?e+=`
#define HIGH_PRECISION`:i.precision==="mediump"?e+=`
#define MEDIUM_PRECISION`:i.precision==="lowp"&&(e+=`
#define LOW_PRECISION`),e}function C_(i){let e="SHADOWMAP_TYPE_BASIC";return i.shadowMapType===Oc?e="SHADOWMAP_TYPE_PCF":i.shadowMapType===Zh?e="SHADOWMAP_TYPE_PCF_SOFT":i.shadowMapType===Zt&&(e="SHADOWMAP_TYPE_VSM"),e}function R_(i){let e="ENVMAP_TYPE_CUBE";if(i.envMap)switch(i.envMapMode){case ui:case hi:e="ENVMAP_TYPE_CUBE";break;case Er:e="ENVMAP_TYPE_CUBE_UV";break}return e}function P_(i){let e="ENVMAP_MODE_REFLECTION";if(i.envMap)switch(i.envMapMode){case hi:e="ENVMAP_MODE_REFRACTION";break}return e}function L_(i){let e="ENVMAP_BLENDING_NONE";if(i.envMap)switch(i.combine){case Fc:e="ENVMAP_BLENDING_MULTIPLY";break;case bd:e="ENVMAP_BLENDING_MIX";break;case Ed:e="ENVMAP_BLENDING_ADD";break}return e}function U_(i){const e=i.envMapCubeUVHeight;if(e===null)return null;const t=Math.log2(e)-2,n=1/e;return{texelWidth:1/(3*Math.max(Math.pow(2,t),7*16)),texelHeight:n,maxMip:t}}function D_(i,e,t,n){const r=i.getContext(),s=t.defines;let o=t.vertexShader,a=t.fragmentShader;const l=C_(t),c=R_(t),u=P_(t),h=L_(t),d=U_(t),f=y_(t),g=S_(s),x=r.createProgram();let m,p,T=t.glslVersion?"#version "+t.glslVersion+`
`:"";t.isRawShaderMaterial?(m=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g].filter(wi).join(`
`),m.length>0&&(m+=`
`),p=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g].filter(wi).join(`
`),p.length>0&&(p+=`
`)):(m=[$l(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g,t.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",t.batching?"#define USE_BATCHING":"",t.batchingColor?"#define USE_BATCHING_COLOR":"",t.instancing?"#define USE_INSTANCING":"",t.instancingColor?"#define USE_INSTANCING_COLOR":"",t.instancingMorph?"#define USE_INSTANCING_MORPH":"",t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.map?"#define USE_MAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+u:"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.displacementMap?"#define USE_DISPLACEMENTMAP":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.mapUv?"#define MAP_UV "+t.mapUv:"",t.alphaMapUv?"#define ALPHAMAP_UV "+t.alphaMapUv:"",t.lightMapUv?"#define LIGHTMAP_UV "+t.lightMapUv:"",t.aoMapUv?"#define AOMAP_UV "+t.aoMapUv:"",t.emissiveMapUv?"#define EMISSIVEMAP_UV "+t.emissiveMapUv:"",t.bumpMapUv?"#define BUMPMAP_UV "+t.bumpMapUv:"",t.normalMapUv?"#define NORMALMAP_UV "+t.normalMapUv:"",t.displacementMapUv?"#define DISPLACEMENTMAP_UV "+t.displacementMapUv:"",t.metalnessMapUv?"#define METALNESSMAP_UV "+t.metalnessMapUv:"",t.roughnessMapUv?"#define ROUGHNESSMAP_UV "+t.roughnessMapUv:"",t.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+t.anisotropyMapUv:"",t.clearcoatMapUv?"#define CLEARCOATMAP_UV "+t.clearcoatMapUv:"",t.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+t.clearcoatNormalMapUv:"",t.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+t.clearcoatRoughnessMapUv:"",t.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+t.iridescenceMapUv:"",t.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+t.iridescenceThicknessMapUv:"",t.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+t.sheenColorMapUv:"",t.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+t.sheenRoughnessMapUv:"",t.specularMapUv?"#define SPECULARMAP_UV "+t.specularMapUv:"",t.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+t.specularColorMapUv:"",t.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+t.specularIntensityMapUv:"",t.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+t.transmissionMapUv:"",t.thicknessMapUv?"#define THICKNESSMAP_UV "+t.thicknessMapUv:"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.flatShading?"#define FLAT_SHADED":"",t.skinning?"#define USE_SKINNING":"",t.morphTargets?"#define USE_MORPHTARGETS":"",t.morphNormals&&t.flatShading===!1?"#define USE_MORPHNORMALS":"",t.morphColors?"#define USE_MORPHCOLORS":"",t.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+t.morphTextureStride:"",t.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+t.morphTargetsCount:"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+l:"",t.sizeAttenuation?"#define USE_SIZEATTENUATION":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(wi).join(`
`),p=[$l(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g,t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",t.map?"#define USE_MAP":"",t.matcap?"#define USE_MATCAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+c:"",t.envMap?"#define "+u:"",t.envMap?"#define "+h:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoat?"#define USE_CLEARCOAT":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.dispersion?"#define USE_DISPERSION":"",t.iridescence?"#define USE_IRIDESCENCE":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaTest?"#define USE_ALPHATEST":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.sheen?"#define USE_SHEEN":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors||t.instancingColor||t.batchingColor?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.gradientMap?"#define USE_GRADIENTMAP":"",t.flatShading?"#define FLAT_SHADED":"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+l:"",t.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",t.toneMapping!==pn?"#define TONE_MAPPING":"",t.toneMapping!==pn?ke.tonemapping_pars_fragment:"",t.toneMapping!==pn?x_("toneMapping",t.toneMapping):"",t.dithering?"#define DITHERING":"",t.opaque?"#define OPAQUE":"",ke.colorspace_pars_fragment,v_("linearToOutputTexel",t.outputColorSpace),t.useDepthPacking?"#define DEPTH_PACKING "+t.depthPacking:"",`
`].filter(wi).join(`
`)),o=va(o),o=ql(o,t),o=Yl(o,t),a=va(a),a=ql(a,t),a=Yl(a,t),o=jl(o),a=jl(a),t.isRawShaderMaterial!==!0&&(T=`#version 300 es
`,m=[f,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+m,p=["#define varying in",t.glslVersion===hl?"":"layout(location = 0) out highp vec4 pc_fragColor;",t.glslVersion===hl?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+p);const S=T+m+o,A=T+p+a,B=Wl(r,r.VERTEX_SHADER,S),D=Wl(r,r.FRAGMENT_SHADER,A);r.attachShader(x,B),r.attachShader(x,D),t.index0AttributeName!==void 0?r.bindAttribLocation(x,0,t.index0AttributeName):t.morphTargets===!0&&r.bindAttribLocation(x,0,"position"),r.linkProgram(x);function P(C){if(i.debug.checkShaderErrors){const H=r.getProgramInfoLog(x).trim(),O=r.getShaderInfoLog(B).trim(),$=r.getShaderInfoLog(D).trim();let V=!0,Y=!0;if(r.getProgramParameter(x,r.LINK_STATUS)===!1)if(V=!1,typeof i.debug.onShaderError=="function")i.debug.onShaderError(r,x,B,D);else{const te=Xl(r,B,"vertex"),v=Xl(r,D,"fragment");console.error("THREE.WebGLProgram: Shader Error "+r.getError()+" - VALIDATE_STATUS "+r.getProgramParameter(x,r.VALIDATE_STATUS)+`

Material Name: `+C.name+`
Material Type: `+C.type+`

Program Info Log: `+H+`
`+te+`
`+v)}else H!==""?console.warn("THREE.WebGLProgram: Program Info Log:",H):(O===""||$==="")&&(Y=!1);Y&&(C.diagnostics={runnable:V,programLog:H,vertexShader:{log:O,prefix:m},fragmentShader:{log:$,prefix:p}})}r.deleteShader(B),r.deleteShader(D),X=new fr(r,x),E=M_(r,x)}let X;this.getUniforms=function(){return X===void 0&&P(this),X};let E;this.getAttributes=function(){return E===void 0&&P(this),E};let M=t.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return M===!1&&(M=r.getProgramParameter(x,p_)),M},this.destroy=function(){n.releaseStatesOfProgram(this),r.deleteProgram(x),this.program=void 0},this.type=t.shaderType,this.name=t.shaderName,this.id=m_++,this.cacheKey=e,this.usedTimes=1,this.program=x,this.vertexShader=B,this.fragmentShader=D,this}let I_=0;class N_{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(e){const t=e.vertexShader,n=e.fragmentShader,r=this._getShaderStage(t),s=this._getShaderStage(n),o=this._getShaderCacheForMaterial(e);return o.has(r)===!1&&(o.add(r),r.usedTimes++),o.has(s)===!1&&(o.add(s),s.usedTimes++),this}remove(e){const t=this.materialCache.get(e);for(const n of t)n.usedTimes--,n.usedTimes===0&&this.shaderCache.delete(n.code);return this.materialCache.delete(e),this}getVertexShaderID(e){return this._getShaderStage(e.vertexShader).id}getFragmentShaderID(e){return this._getShaderStage(e.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(e){const t=this.materialCache;let n=t.get(e);return n===void 0&&(n=new Set,t.set(e,n)),n}_getShaderStage(e){const t=this.shaderCache;let n=t.get(e);return n===void 0&&(n=new O_(e),t.set(e,n)),n}}class O_{constructor(e){this.id=I_++,this.code=e,this.usedTimes=0}}function F_(i,e,t,n,r,s,o){const a=new Zc,l=new N_,c=new Set,u=[],h=r.logarithmicDepthBuffer,d=r.vertexTextures;let f=r.precision;const g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function x(E){return c.add(E),E===0?"uv":`uv${E}`}function m(E,M,C,H,O){const $=H.fog,V=O.geometry,Y=E.isMeshStandardMaterial?H.environment:null,te=(E.isMeshStandardMaterial?t:e).get(E.envMap||Y),v=te&&te.mapping===Er?te.image.height:null,w=g[E.type];E.precision!==null&&(f=r.getMaxPrecision(E.precision),f!==E.precision&&console.warn("THREE.WebGLProgram.getParameters:",E.precision,"not supported, using",f,"instead."));const U=V.morphAttributes.position||V.morphAttributes.normal||V.morphAttributes.color,N=U!==void 0?U.length:0;let G=0;V.morphAttributes.position!==void 0&&(G=1),V.morphAttributes.normal!==void 0&&(G=2),V.morphAttributes.color!==void 0&&(G=3);let K,I,k,j;if(w){const We=kt[w];K=We.vertexShader,I=We.fragmentShader}else K=E.vertexShader,I=E.fragmentShader,l.update(E),k=l.getVertexShaderID(E),j=l.getFragmentShaderID(E);const q=i.getRenderTarget(),le=O.isInstancedMesh===!0,ve=O.isBatchedMesh===!0,ge=!!E.map,L=!!E.matcap,xe=!!te,Te=!!E.aoMap,Xe=!!E.lightMap,Me=!!E.bumpMap,Be=!!E.normalMap,Ie=!!E.displacementMap,Ae=!!E.emissiveMap,Ve=!!E.metalnessMap,R=!!E.roughnessMap,y=E.anisotropy>0,ee=E.clearcoat>0,re=E.dispersion>0,ae=E.iridescence>0,oe=E.sheen>0,be=E.transmission>0,de=y&&!!E.anisotropyMap,he=ee&&!!E.clearcoatMap,Ne=ee&&!!E.clearcoatNormalMap,ce=ee&&!!E.clearcoatRoughnessMap,ye=ae&&!!E.iridescenceMap,Ge=ae&&!!E.iridescenceThicknessMap,Re=oe&&!!E.sheenColorMap,pe=oe&&!!E.sheenRoughnessMap,Oe=!!E.specularMap,Fe=!!E.specularColorMap,Qe=!!E.specularIntensityMap,_=be&&!!E.transmissionMap,ne=be&&!!E.thicknessMap,Z=!!E.gradientMap,ie=!!E.alphaMap,se=E.alphaTest>0,Ee=!!E.alphaHash,Ue=!!E.extensions;let et=pn;E.toneMapped&&(q===null||q.isXRRenderTarget===!0)&&(et=i.toneMapping);const it={shaderID:w,shaderType:E.type,shaderName:E.name,vertexShader:K,fragmentShader:I,defines:E.defines,customVertexShaderID:k,customFragmentShaderID:j,isRawShaderMaterial:E.isRawShaderMaterial===!0,glslVersion:E.glslVersion,precision:f,batching:ve,batchingColor:ve&&O._colorsTexture!==null,instancing:le,instancingColor:le&&O.instanceColor!==null,instancingMorph:le&&O.morphTexture!==null,supportsVertexTextures:d,outputColorSpace:q===null?i.outputColorSpace:q.isXRRenderTarget===!0?q.texture.colorSpace:vn,alphaToCoverage:!!E.alphaToCoverage,map:ge,matcap:L,envMap:xe,envMapMode:xe&&te.mapping,envMapCubeUVHeight:v,aoMap:Te,lightMap:Xe,bumpMap:Me,normalMap:Be,displacementMap:d&&Ie,emissiveMap:Ae,normalMapObjectSpace:Be&&E.normalMapType===Xd,normalMapTangentSpace:Be&&E.normalMapType===Xc,metalnessMap:Ve,roughnessMap:R,anisotropy:y,anisotropyMap:de,clearcoat:ee,clearcoatMap:he,clearcoatNormalMap:Ne,clearcoatRoughnessMap:ce,dispersion:re,iridescence:ae,iridescenceMap:ye,iridescenceThicknessMap:Ge,sheen:oe,sheenColorMap:Re,sheenRoughnessMap:pe,specularMap:Oe,specularColorMap:Fe,specularIntensityMap:Qe,transmission:be,transmissionMap:_,thicknessMap:ne,gradientMap:Z,opaque:E.transparent===!1&&E.blending===oi&&E.alphaToCoverage===!1,alphaMap:ie,alphaTest:se,alphaHash:Ee,combine:E.combine,mapUv:ge&&x(E.map.channel),aoMapUv:Te&&x(E.aoMap.channel),lightMapUv:Xe&&x(E.lightMap.channel),bumpMapUv:Me&&x(E.bumpMap.channel),normalMapUv:Be&&x(E.normalMap.channel),displacementMapUv:Ie&&x(E.displacementMap.channel),emissiveMapUv:Ae&&x(E.emissiveMap.channel),metalnessMapUv:Ve&&x(E.metalnessMap.channel),roughnessMapUv:R&&x(E.roughnessMap.channel),anisotropyMapUv:de&&x(E.anisotropyMap.channel),clearcoatMapUv:he&&x(E.clearcoatMap.channel),clearcoatNormalMapUv:Ne&&x(E.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:ce&&x(E.clearcoatRoughnessMap.channel),iridescenceMapUv:ye&&x(E.iridescenceMap.channel),iridescenceThicknessMapUv:Ge&&x(E.iridescenceThicknessMap.channel),sheenColorMapUv:Re&&x(E.sheenColorMap.channel),sheenRoughnessMapUv:pe&&x(E.sheenRoughnessMap.channel),specularMapUv:Oe&&x(E.specularMap.channel),specularColorMapUv:Fe&&x(E.specularColorMap.channel),specularIntensityMapUv:Qe&&x(E.specularIntensityMap.channel),transmissionMapUv:_&&x(E.transmissionMap.channel),thicknessMapUv:ne&&x(E.thicknessMap.channel),alphaMapUv:ie&&x(E.alphaMap.channel),vertexTangents:!!V.attributes.tangent&&(Be||y),vertexColors:E.vertexColors,vertexAlphas:E.vertexColors===!0&&!!V.attributes.color&&V.attributes.color.itemSize===4,pointsUvs:O.isPoints===!0&&!!V.attributes.uv&&(ge||ie),fog:!!$,useFog:E.fog===!0,fogExp2:!!$&&$.isFogExp2,flatShading:E.flatShading===!0,sizeAttenuation:E.sizeAttenuation===!0,logarithmicDepthBuffer:h,skinning:O.isSkinnedMesh===!0,morphTargets:V.morphAttributes.position!==void 0,morphNormals:V.morphAttributes.normal!==void 0,morphColors:V.morphAttributes.color!==void 0,morphTargetsCount:N,morphTextureStride:G,numDirLights:M.directional.length,numPointLights:M.point.length,numSpotLights:M.spot.length,numSpotLightMaps:M.spotLightMap.length,numRectAreaLights:M.rectArea.length,numHemiLights:M.hemi.length,numDirLightShadows:M.directionalShadowMap.length,numPointLightShadows:M.pointShadowMap.length,numSpotLightShadows:M.spotShadowMap.length,numSpotLightShadowsWithMaps:M.numSpotLightShadowsWithMaps,numLightProbes:M.numLightProbes,numClippingPlanes:o.numPlanes,numClipIntersection:o.numIntersection,dithering:E.dithering,shadowMapEnabled:i.shadowMap.enabled&&C.length>0,shadowMapType:i.shadowMap.type,toneMapping:et,decodeVideoTexture:ge&&E.map.isVideoTexture===!0&&je.getTransfer(E.map.colorSpace)===Ze,premultipliedAlpha:E.premultipliedAlpha,doubleSided:E.side===Nt,flipSided:E.side===yt,useDepthPacking:E.depthPacking>=0,depthPacking:E.depthPacking||0,index0AttributeName:E.index0AttributeName,extensionClipCullDistance:Ue&&E.extensions.clipCullDistance===!0&&n.has("WEBGL_clip_cull_distance"),extensionMultiDraw:Ue&&E.extensions.multiDraw===!0&&n.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:n.has("KHR_parallel_shader_compile"),customProgramCacheKey:E.customProgramCacheKey()};return it.vertexUv1s=c.has(1),it.vertexUv2s=c.has(2),it.vertexUv3s=c.has(3),c.clear(),it}function p(E){const M=[];if(E.shaderID?M.push(E.shaderID):(M.push(E.customVertexShaderID),M.push(E.customFragmentShaderID)),E.defines!==void 0)for(const C in E.defines)M.push(C),M.push(E.defines[C]);return E.isRawShaderMaterial===!1&&(T(M,E),S(M,E),M.push(i.outputColorSpace)),M.push(E.customProgramCacheKey),M.join()}function T(E,M){E.push(M.precision),E.push(M.outputColorSpace),E.push(M.envMapMode),E.push(M.envMapCubeUVHeight),E.push(M.mapUv),E.push(M.alphaMapUv),E.push(M.lightMapUv),E.push(M.aoMapUv),E.push(M.bumpMapUv),E.push(M.normalMapUv),E.push(M.displacementMapUv),E.push(M.emissiveMapUv),E.push(M.metalnessMapUv),E.push(M.roughnessMapUv),E.push(M.anisotropyMapUv),E.push(M.clearcoatMapUv),E.push(M.clearcoatNormalMapUv),E.push(M.clearcoatRoughnessMapUv),E.push(M.iridescenceMapUv),E.push(M.iridescenceThicknessMapUv),E.push(M.sheenColorMapUv),E.push(M.sheenRoughnessMapUv),E.push(M.specularMapUv),E.push(M.specularColorMapUv),E.push(M.specularIntensityMapUv),E.push(M.transmissionMapUv),E.push(M.thicknessMapUv),E.push(M.combine),E.push(M.fogExp2),E.push(M.sizeAttenuation),E.push(M.morphTargetsCount),E.push(M.morphAttributeCount),E.push(M.numDirLights),E.push(M.numPointLights),E.push(M.numSpotLights),E.push(M.numSpotLightMaps),E.push(M.numHemiLights),E.push(M.numRectAreaLights),E.push(M.numDirLightShadows),E.push(M.numPointLightShadows),E.push(M.numSpotLightShadows),E.push(M.numSpotLightShadowsWithMaps),E.push(M.numLightProbes),E.push(M.shadowMapType),E.push(M.toneMapping),E.push(M.numClippingPlanes),E.push(M.numClipIntersection),E.push(M.depthPacking)}function S(E,M){a.disableAll(),M.supportsVertexTextures&&a.enable(0),M.instancing&&a.enable(1),M.instancingColor&&a.enable(2),M.instancingMorph&&a.enable(3),M.matcap&&a.enable(4),M.envMap&&a.enable(5),M.normalMapObjectSpace&&a.enable(6),M.normalMapTangentSpace&&a.enable(7),M.clearcoat&&a.enable(8),M.iridescence&&a.enable(9),M.alphaTest&&a.enable(10),M.vertexColors&&a.enable(11),M.vertexAlphas&&a.enable(12),M.vertexUv1s&&a.enable(13),M.vertexUv2s&&a.enable(14),M.vertexUv3s&&a.enable(15),M.vertexTangents&&a.enable(16),M.anisotropy&&a.enable(17),M.alphaHash&&a.enable(18),M.batching&&a.enable(19),M.dispersion&&a.enable(20),M.batchingColor&&a.enable(21),E.push(a.mask),a.disableAll(),M.fog&&a.enable(0),M.useFog&&a.enable(1),M.flatShading&&a.enable(2),M.logarithmicDepthBuffer&&a.enable(3),M.skinning&&a.enable(4),M.morphTargets&&a.enable(5),M.morphNormals&&a.enable(6),M.morphColors&&a.enable(7),M.premultipliedAlpha&&a.enable(8),M.shadowMapEnabled&&a.enable(9),M.doubleSided&&a.enable(10),M.flipSided&&a.enable(11),M.useDepthPacking&&a.enable(12),M.dithering&&a.enable(13),M.transmission&&a.enable(14),M.sheen&&a.enable(15),M.opaque&&a.enable(16),M.pointsUvs&&a.enable(17),M.decodeVideoTexture&&a.enable(18),M.alphaToCoverage&&a.enable(19),E.push(a.mask)}function A(E){const M=g[E.type];let C;if(M){const H=kt[M];C=Sf.clone(H.uniforms)}else C=E.uniforms;return C}function B(E,M){let C;for(let H=0,O=u.length;H<O;H++){const $=u[H];if($.cacheKey===M){C=$,++C.usedTimes;break}}return C===void 0&&(C=new D_(i,M,E,s),u.push(C)),C}function D(E){if(--E.usedTimes===0){const M=u.indexOf(E);u[M]=u[u.length-1],u.pop(),E.destroy()}}function P(E){l.remove(E)}function X(){l.dispose()}return{getParameters:m,getProgramCacheKey:p,getUniforms:A,acquireProgram:B,releaseProgram:D,releaseShaderCache:P,programs:u,dispose:X}}function B_(){let i=new WeakMap;function e(s){let o=i.get(s);return o===void 0&&(o={},i.set(s,o)),o}function t(s){i.delete(s)}function n(s,o,a){i.get(s)[o]=a}function r(){i=new WeakMap}return{get:e,remove:t,update:n,dispose:r}}function k_(i,e){return i.groupOrder!==e.groupOrder?i.groupOrder-e.groupOrder:i.renderOrder!==e.renderOrder?i.renderOrder-e.renderOrder:i.material.id!==e.material.id?i.material.id-e.material.id:i.z!==e.z?i.z-e.z:i.id-e.id}function Kl(i,e){return i.groupOrder!==e.groupOrder?i.groupOrder-e.groupOrder:i.renderOrder!==e.renderOrder?i.renderOrder-e.renderOrder:i.z!==e.z?e.z-i.z:i.id-e.id}function Zl(){const i=[];let e=0;const t=[],n=[],r=[];function s(){e=0,t.length=0,n.length=0,r.length=0}function o(h,d,f,g,x,m){let p=i[e];return p===void 0?(p={id:h.id,object:h,geometry:d,material:f,groupOrder:g,renderOrder:h.renderOrder,z:x,group:m},i[e]=p):(p.id=h.id,p.object=h,p.geometry=d,p.material=f,p.groupOrder=g,p.renderOrder=h.renderOrder,p.z=x,p.group=m),e++,p}function a(h,d,f,g,x,m){const p=o(h,d,f,g,x,m);f.transmission>0?n.push(p):f.transparent===!0?r.push(p):t.push(p)}function l(h,d,f,g,x,m){const p=o(h,d,f,g,x,m);f.transmission>0?n.unshift(p):f.transparent===!0?r.unshift(p):t.unshift(p)}function c(h,d){t.length>1&&t.sort(h||k_),n.length>1&&n.sort(d||Kl),r.length>1&&r.sort(d||Kl)}function u(){for(let h=e,d=i.length;h<d;h++){const f=i[h];if(f.id===null)break;f.id=null,f.object=null,f.geometry=null,f.material=null,f.group=null}}return{opaque:t,transmissive:n,transparent:r,init:s,push:a,unshift:l,finish:u,sort:c}}function z_(){let i=new WeakMap;function e(n,r){const s=i.get(n);let o;return s===void 0?(o=new Zl,i.set(n,[o])):r>=s.length?(o=new Zl,s.push(o)):o=s[r],o}function t(){i=new WeakMap}return{get:e,dispose:t}}function V_(){const i={};return{get:function(e){if(i[e.id]!==void 0)return i[e.id];let t;switch(e.type){case"DirectionalLight":t={direction:new z,color:new He};break;case"SpotLight":t={position:new z,direction:new z,color:new He,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":t={position:new z,color:new He,distance:0,decay:0};break;case"HemisphereLight":t={direction:new z,skyColor:new He,groundColor:new He};break;case"RectAreaLight":t={color:new He,position:new z,halfWidth:new z,halfHeight:new z};break}return i[e.id]=t,t}}}function G_(){const i={};return{get:function(e){if(i[e.id]!==void 0)return i[e.id];let t;switch(e.type){case"DirectionalLight":t={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new De};break;case"SpotLight":t={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new De};break;case"PointLight":t={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new De,shadowCameraNear:1,shadowCameraFar:1e3};break}return i[e.id]=t,t}}}let H_=0;function W_(i,e){return(e.castShadow?2:0)-(i.castShadow?2:0)+(e.map?1:0)-(i.map?1:0)}function X_(i){const e=new V_,t=G_(),n={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let c=0;c<9;c++)n.probe.push(new z);const r=new z,s=new Je,o=new Je;function a(c){let u=0,h=0,d=0;for(let E=0;E<9;E++)n.probe[E].set(0,0,0);let f=0,g=0,x=0,m=0,p=0,T=0,S=0,A=0,B=0,D=0,P=0;c.sort(W_);for(let E=0,M=c.length;E<M;E++){const C=c[E],H=C.color,O=C.intensity,$=C.distance,V=C.shadow&&C.shadow.map?C.shadow.map.texture:null;if(C.isAmbientLight)u+=H.r*O,h+=H.g*O,d+=H.b*O;else if(C.isLightProbe){for(let Y=0;Y<9;Y++)n.probe[Y].addScaledVector(C.sh.coefficients[Y],O);P++}else if(C.isDirectionalLight){const Y=e.get(C);if(Y.color.copy(C.color).multiplyScalar(C.intensity),C.castShadow){const te=C.shadow,v=t.get(C);v.shadowBias=te.bias,v.shadowNormalBias=te.normalBias,v.shadowRadius=te.radius,v.shadowMapSize=te.mapSize,n.directionalShadow[f]=v,n.directionalShadowMap[f]=V,n.directionalShadowMatrix[f]=C.shadow.matrix,T++}n.directional[f]=Y,f++}else if(C.isSpotLight){const Y=e.get(C);Y.position.setFromMatrixPosition(C.matrixWorld),Y.color.copy(H).multiplyScalar(O),Y.distance=$,Y.coneCos=Math.cos(C.angle),Y.penumbraCos=Math.cos(C.angle*(1-C.penumbra)),Y.decay=C.decay,n.spot[x]=Y;const te=C.shadow;if(C.map&&(n.spotLightMap[B]=C.map,B++,te.updateMatrices(C),C.castShadow&&D++),n.spotLightMatrix[x]=te.matrix,C.castShadow){const v=t.get(C);v.shadowBias=te.bias,v.shadowNormalBias=te.normalBias,v.shadowRadius=te.radius,v.shadowMapSize=te.mapSize,n.spotShadow[x]=v,n.spotShadowMap[x]=V,A++}x++}else if(C.isRectAreaLight){const Y=e.get(C);Y.color.copy(H).multiplyScalar(O),Y.halfWidth.set(C.width*.5,0,0),Y.halfHeight.set(0,C.height*.5,0),n.rectArea[m]=Y,m++}else if(C.isPointLight){const Y=e.get(C);if(Y.color.copy(C.color).multiplyScalar(C.intensity),Y.distance=C.distance,Y.decay=C.decay,C.castShadow){const te=C.shadow,v=t.get(C);v.shadowBias=te.bias,v.shadowNormalBias=te.normalBias,v.shadowRadius=te.radius,v.shadowMapSize=te.mapSize,v.shadowCameraNear=te.camera.near,v.shadowCameraFar=te.camera.far,n.pointShadow[g]=v,n.pointShadowMap[g]=V,n.pointShadowMatrix[g]=C.shadow.matrix,S++}n.point[g]=Y,g++}else if(C.isHemisphereLight){const Y=e.get(C);Y.skyColor.copy(C.color).multiplyScalar(O),Y.groundColor.copy(C.groundColor).multiplyScalar(O),n.hemi[p]=Y,p++}}m>0&&(i.has("OES_texture_float_linear")===!0?(n.rectAreaLTC1=fe.LTC_FLOAT_1,n.rectAreaLTC2=fe.LTC_FLOAT_2):(n.rectAreaLTC1=fe.LTC_HALF_1,n.rectAreaLTC2=fe.LTC_HALF_2)),n.ambient[0]=u,n.ambient[1]=h,n.ambient[2]=d;const X=n.hash;(X.directionalLength!==f||X.pointLength!==g||X.spotLength!==x||X.rectAreaLength!==m||X.hemiLength!==p||X.numDirectionalShadows!==T||X.numPointShadows!==S||X.numSpotShadows!==A||X.numSpotMaps!==B||X.numLightProbes!==P)&&(n.directional.length=f,n.spot.length=x,n.rectArea.length=m,n.point.length=g,n.hemi.length=p,n.directionalShadow.length=T,n.directionalShadowMap.length=T,n.pointShadow.length=S,n.pointShadowMap.length=S,n.spotShadow.length=A,n.spotShadowMap.length=A,n.directionalShadowMatrix.length=T,n.pointShadowMatrix.length=S,n.spotLightMatrix.length=A+B-D,n.spotLightMap.length=B,n.numSpotLightShadowsWithMaps=D,n.numLightProbes=P,X.directionalLength=f,X.pointLength=g,X.spotLength=x,X.rectAreaLength=m,X.hemiLength=p,X.numDirectionalShadows=T,X.numPointShadows=S,X.numSpotShadows=A,X.numSpotMaps=B,X.numLightProbes=P,n.version=H_++)}function l(c,u){let h=0,d=0,f=0,g=0,x=0;const m=u.matrixWorldInverse;for(let p=0,T=c.length;p<T;p++){const S=c[p];if(S.isDirectionalLight){const A=n.directional[h];A.direction.setFromMatrixPosition(S.matrixWorld),r.setFromMatrixPosition(S.target.matrixWorld),A.direction.sub(r),A.direction.transformDirection(m),h++}else if(S.isSpotLight){const A=n.spot[f];A.position.setFromMatrixPosition(S.matrixWorld),A.position.applyMatrix4(m),A.direction.setFromMatrixPosition(S.matrixWorld),r.setFromMatrixPosition(S.target.matrixWorld),A.direction.sub(r),A.direction.transformDirection(m),f++}else if(S.isRectAreaLight){const A=n.rectArea[g];A.position.setFromMatrixPosition(S.matrixWorld),A.position.applyMatrix4(m),o.identity(),s.copy(S.matrixWorld),s.premultiply(m),o.extractRotation(s),A.halfWidth.set(S.width*.5,0,0),A.halfHeight.set(0,S.height*.5,0),A.halfWidth.applyMatrix4(o),A.halfHeight.applyMatrix4(o),g++}else if(S.isPointLight){const A=n.point[d];A.position.setFromMatrixPosition(S.matrixWorld),A.position.applyMatrix4(m),d++}else if(S.isHemisphereLight){const A=n.hemi[x];A.direction.setFromMatrixPosition(S.matrixWorld),A.direction.transformDirection(m),x++}}}return{setup:a,setupView:l,state:n}}function Jl(i){const e=new X_(i),t=[],n=[];function r(u){c.camera=u,t.length=0,n.length=0}function s(u){t.push(u)}function o(u){n.push(u)}function a(){e.setup(t)}function l(u){e.setupView(t,u)}const c={lightsArray:t,shadowsArray:n,camera:null,lights:e,transmissionRenderTarget:{}};return{init:r,state:c,setupLights:a,setupLightsView:l,pushLight:s,pushShadow:o}}function q_(i){let e=new WeakMap;function t(r,s=0){const o=e.get(r);let a;return o===void 0?(a=new Jl(i),e.set(r,[a])):s>=o.length?(a=new Jl(i),o.push(a)):a=o[s],a}function n(){e=new WeakMap}return{get:t,dispose:n}}class Y_ extends kn{constructor(e){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=Hd,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(e)}copy(e){return super.copy(e),this.depthPacking=e.depthPacking,this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this}}class j_ extends kn{constructor(e){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(e)}copy(e){return super.copy(e),this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this}}const $_=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,K_=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function Z_(i,e,t){let n=new Aa;const r=new De,s=new De,o=new ht,a=new Y_({depthPacking:Wd}),l=new j_,c={},u=t.maxTextureSize,h={[mn]:yt,[yt]:mn,[Nt]:Nt},d=new _n({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new De},radius:{value:4}},vertexShader:$_,fragmentShader:K_}),f=d.clone();f.defines.HORIZONTAL_PASS=1;const g=new Lt;g.setAttribute("position",new Gt(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const x=new Ft(g,d),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=Oc;let p=this.type;this.render=function(D,P,X){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||D.length===0)return;const E=i.getRenderTarget(),M=i.getActiveCubeFace(),C=i.getActiveMipmapLevel(),H=i.state;H.setBlending(fn),H.buffers.color.setClear(1,1,1,1),H.buffers.depth.setTest(!0),H.setScissorTest(!1);const O=p!==Zt&&this.type===Zt,$=p===Zt&&this.type!==Zt;for(let V=0,Y=D.length;V<Y;V++){const te=D[V],v=te.shadow;if(v===void 0){console.warn("THREE.WebGLShadowMap:",te,"has no shadow.");continue}if(v.autoUpdate===!1&&v.needsUpdate===!1)continue;r.copy(v.mapSize);const w=v.getFrameExtents();if(r.multiply(w),s.copy(v.mapSize),(r.x>u||r.y>u)&&(r.x>u&&(s.x=Math.floor(u/w.x),r.x=s.x*w.x,v.mapSize.x=s.x),r.y>u&&(s.y=Math.floor(u/w.y),r.y=s.y*w.y,v.mapSize.y=s.y)),v.map===null||O===!0||$===!0){const N=this.type!==Zt?{minFilter:Pt,magFilter:Pt}:{};v.map!==null&&v.map.dispose(),v.map=new Dn(r.x,r.y,N),v.map.texture.name=te.name+".shadowMap",v.camera.updateProjectionMatrix()}i.setRenderTarget(v.map),i.clear();const U=v.getViewportCount();for(let N=0;N<U;N++){const G=v.getViewport(N);o.set(s.x*G.x,s.y*G.y,s.x*G.z,s.y*G.w),H.viewport(o),v.updateMatrices(te,N),n=v.getFrustum(),A(P,X,v.camera,te,this.type)}v.isPointLightShadow!==!0&&this.type===Zt&&T(v,X),v.needsUpdate=!1}p=this.type,m.needsUpdate=!1,i.setRenderTarget(E,M,C)};function T(D,P){const X=e.update(x);d.defines.VSM_SAMPLES!==D.blurSamples&&(d.defines.VSM_SAMPLES=D.blurSamples,f.defines.VSM_SAMPLES=D.blurSamples,d.needsUpdate=!0,f.needsUpdate=!0),D.mapPass===null&&(D.mapPass=new Dn(r.x,r.y)),d.uniforms.shadow_pass.value=D.map.texture,d.uniforms.resolution.value=D.mapSize,d.uniforms.radius.value=D.radius,i.setRenderTarget(D.mapPass),i.clear(),i.renderBufferDirect(P,null,X,d,x,null),f.uniforms.shadow_pass.value=D.mapPass.texture,f.uniforms.resolution.value=D.mapSize,f.uniforms.radius.value=D.radius,i.setRenderTarget(D.map),i.clear(),i.renderBufferDirect(P,null,X,f,x,null)}function S(D,P,X,E){let M=null;const C=X.isPointLight===!0?D.customDistanceMaterial:D.customDepthMaterial;if(C!==void 0)M=C;else if(M=X.isPointLight===!0?l:a,i.localClippingEnabled&&P.clipShadows===!0&&Array.isArray(P.clippingPlanes)&&P.clippingPlanes.length!==0||P.displacementMap&&P.displacementScale!==0||P.alphaMap&&P.alphaTest>0||P.map&&P.alphaTest>0){const H=M.uuid,O=P.uuid;let $=c[H];$===void 0&&($={},c[H]=$);let V=$[O];V===void 0&&(V=M.clone(),$[O]=V,P.addEventListener("dispose",B)),M=V}if(M.visible=P.visible,M.wireframe=P.wireframe,E===Zt?M.side=P.shadowSide!==null?P.shadowSide:P.side:M.side=P.shadowSide!==null?P.shadowSide:h[P.side],M.alphaMap=P.alphaMap,M.alphaTest=P.alphaTest,M.map=P.map,M.clipShadows=P.clipShadows,M.clippingPlanes=P.clippingPlanes,M.clipIntersection=P.clipIntersection,M.displacementMap=P.displacementMap,M.displacementScale=P.displacementScale,M.displacementBias=P.displacementBias,M.wireframeLinewidth=P.wireframeLinewidth,M.linewidth=P.linewidth,X.isPointLight===!0&&M.isMeshDistanceMaterial===!0){const H=i.properties.get(M);H.light=X}return M}function A(D,P,X,E,M){if(D.visible===!1)return;if(D.layers.test(P.layers)&&(D.isMesh||D.isLine||D.isPoints)&&(D.castShadow||D.receiveShadow&&M===Zt)&&(!D.frustumCulled||n.intersectsObject(D))){D.modelViewMatrix.multiplyMatrices(X.matrixWorldInverse,D.matrixWorld);const O=e.update(D),$=D.material;if(Array.isArray($)){const V=O.groups;for(let Y=0,te=V.length;Y<te;Y++){const v=V[Y],w=$[v.materialIndex];if(w&&w.visible){const U=S(D,w,E,M);D.onBeforeShadow(i,D,P,X,O,U,v),i.renderBufferDirect(X,null,O,U,D,v),D.onAfterShadow(i,D,P,X,O,U,v)}}}else if($.visible){const V=S(D,$,E,M);D.onBeforeShadow(i,D,P,X,O,V,null),i.renderBufferDirect(X,null,O,V,D,null),D.onAfterShadow(i,D,P,X,O,V,null)}}const H=D.children;for(let O=0,$=H.length;O<$;O++)A(H[O],P,X,E,M)}function B(D){D.target.removeEventListener("dispose",B);for(const X in c){const E=c[X],M=D.target.uuid;M in E&&(E[M].dispose(),delete E[M])}}}function J_(i){function e(){let _=!1;const ne=new ht;let Z=null;const ie=new ht(0,0,0,0);return{setMask:function(se){Z!==se&&!_&&(i.colorMask(se,se,se,se),Z=se)},setLocked:function(se){_=se},setClear:function(se,Ee,Ue,et,it){it===!0&&(se*=et,Ee*=et,Ue*=et),ne.set(se,Ee,Ue,et),ie.equals(ne)===!1&&(i.clearColor(se,Ee,Ue,et),ie.copy(ne))},reset:function(){_=!1,Z=null,ie.set(-1,0,0,0)}}}function t(){let _=!1,ne=null,Z=null,ie=null;return{setTest:function(se){se?j(i.DEPTH_TEST):q(i.DEPTH_TEST)},setMask:function(se){ne!==se&&!_&&(i.depthMask(se),ne=se)},setFunc:function(se){if(Z!==se){switch(se){case gd:i.depthFunc(i.NEVER);break;case _d:i.depthFunc(i.ALWAYS);break;case vd:i.depthFunc(i.LESS);break;case pr:i.depthFunc(i.LEQUAL);break;case xd:i.depthFunc(i.EQUAL);break;case yd:i.depthFunc(i.GEQUAL);break;case Sd:i.depthFunc(i.GREATER);break;case Md:i.depthFunc(i.NOTEQUAL);break;default:i.depthFunc(i.LEQUAL)}Z=se}},setLocked:function(se){_=se},setClear:function(se){ie!==se&&(i.clearDepth(se),ie=se)},reset:function(){_=!1,ne=null,Z=null,ie=null}}}function n(){let _=!1,ne=null,Z=null,ie=null,se=null,Ee=null,Ue=null,et=null,it=null;return{setTest:function(We){_||(We?j(i.STENCIL_TEST):q(i.STENCIL_TEST))},setMask:function(We){ne!==We&&!_&&(i.stencilMask(We),ne=We)},setFunc:function(We,rt,st){(Z!==We||ie!==rt||se!==st)&&(i.stencilFunc(We,rt,st),Z=We,ie=rt,se=st)},setOp:function(We,rt,st){(Ee!==We||Ue!==rt||et!==st)&&(i.stencilOp(We,rt,st),Ee=We,Ue=rt,et=st)},setLocked:function(We){_=We},setClear:function(We){it!==We&&(i.clearStencil(We),it=We)},reset:function(){_=!1,ne=null,Z=null,ie=null,se=null,Ee=null,Ue=null,et=null,it=null}}}const r=new e,s=new t,o=new n,a=new WeakMap,l=new WeakMap;let c={},u={},h=new WeakMap,d=[],f=null,g=!1,x=null,m=null,p=null,T=null,S=null,A=null,B=null,D=new He(0,0,0),P=0,X=!1,E=null,M=null,C=null,H=null,O=null;const $=i.getParameter(i.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let V=!1,Y=0;const te=i.getParameter(i.VERSION);te.indexOf("WebGL")!==-1?(Y=parseFloat(/^WebGL (\d)/.exec(te)[1]),V=Y>=1):te.indexOf("OpenGL ES")!==-1&&(Y=parseFloat(/^OpenGL ES (\d)/.exec(te)[1]),V=Y>=2);let v=null,w={};const U=i.getParameter(i.SCISSOR_BOX),N=i.getParameter(i.VIEWPORT),G=new ht().fromArray(U),K=new ht().fromArray(N);function I(_,ne,Z,ie){const se=new Uint8Array(4),Ee=i.createTexture();i.bindTexture(_,Ee),i.texParameteri(_,i.TEXTURE_MIN_FILTER,i.NEAREST),i.texParameteri(_,i.TEXTURE_MAG_FILTER,i.NEAREST);for(let Ue=0;Ue<Z;Ue++)_===i.TEXTURE_3D||_===i.TEXTURE_2D_ARRAY?i.texImage3D(ne,0,i.RGBA,1,1,ie,0,i.RGBA,i.UNSIGNED_BYTE,se):i.texImage2D(ne+Ue,0,i.RGBA,1,1,0,i.RGBA,i.UNSIGNED_BYTE,se);return Ee}const k={};k[i.TEXTURE_2D]=I(i.TEXTURE_2D,i.TEXTURE_2D,1),k[i.TEXTURE_CUBE_MAP]=I(i.TEXTURE_CUBE_MAP,i.TEXTURE_CUBE_MAP_POSITIVE_X,6),k[i.TEXTURE_2D_ARRAY]=I(i.TEXTURE_2D_ARRAY,i.TEXTURE_2D_ARRAY,1,1),k[i.TEXTURE_3D]=I(i.TEXTURE_3D,i.TEXTURE_3D,1,1),r.setClear(0,0,0,1),s.setClear(1),o.setClear(0),j(i.DEPTH_TEST),s.setFunc(pr),Me(!1),Be(Do),j(i.CULL_FACE),Te(fn);function j(_){c[_]!==!0&&(i.enable(_),c[_]=!0)}function q(_){c[_]!==!1&&(i.disable(_),c[_]=!1)}function le(_,ne){return u[_]!==ne?(i.bindFramebuffer(_,ne),u[_]=ne,_===i.DRAW_FRAMEBUFFER&&(u[i.FRAMEBUFFER]=ne),_===i.FRAMEBUFFER&&(u[i.DRAW_FRAMEBUFFER]=ne),!0):!1}function ve(_,ne){let Z=d,ie=!1;if(_){Z=h.get(ne),Z===void 0&&(Z=[],h.set(ne,Z));const se=_.textures;if(Z.length!==se.length||Z[0]!==i.COLOR_ATTACHMENT0){for(let Ee=0,Ue=se.length;Ee<Ue;Ee++)Z[Ee]=i.COLOR_ATTACHMENT0+Ee;Z.length=se.length,ie=!0}}else Z[0]!==i.BACK&&(Z[0]=i.BACK,ie=!0);ie&&i.drawBuffers(Z)}function ge(_){return f!==_?(i.useProgram(_),f=_,!0):!1}const L={[Rn]:i.FUNC_ADD,[Qh]:i.FUNC_SUBTRACT,[ed]:i.FUNC_REVERSE_SUBTRACT};L[td]=i.MIN,L[nd]=i.MAX;const xe={[id]:i.ZERO,[rd]:i.ONE,[sd]:i.SRC_COLOR,[ha]:i.SRC_ALPHA,[hd]:i.SRC_ALPHA_SATURATE,[cd]:i.DST_COLOR,[od]:i.DST_ALPHA,[ad]:i.ONE_MINUS_SRC_COLOR,[da]:i.ONE_MINUS_SRC_ALPHA,[ud]:i.ONE_MINUS_DST_COLOR,[ld]:i.ONE_MINUS_DST_ALPHA,[dd]:i.CONSTANT_COLOR,[fd]:i.ONE_MINUS_CONSTANT_COLOR,[pd]:i.CONSTANT_ALPHA,[md]:i.ONE_MINUS_CONSTANT_ALPHA};function Te(_,ne,Z,ie,se,Ee,Ue,et,it,We){if(_===fn){g===!0&&(q(i.BLEND),g=!1);return}if(g===!1&&(j(i.BLEND),g=!0),_!==Jh){if(_!==x||We!==X){if((m!==Rn||S!==Rn)&&(i.blendEquation(i.FUNC_ADD),m=Rn,S=Rn),We)switch(_){case oi:i.blendFuncSeparate(i.ONE,i.ONE_MINUS_SRC_ALPHA,i.ONE,i.ONE_MINUS_SRC_ALPHA);break;case Io:i.blendFunc(i.ONE,i.ONE);break;case No:i.blendFuncSeparate(i.ZERO,i.ONE_MINUS_SRC_COLOR,i.ZERO,i.ONE);break;case Oo:i.blendFuncSeparate(i.ZERO,i.SRC_COLOR,i.ZERO,i.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",_);break}else switch(_){case oi:i.blendFuncSeparate(i.SRC_ALPHA,i.ONE_MINUS_SRC_ALPHA,i.ONE,i.ONE_MINUS_SRC_ALPHA);break;case Io:i.blendFunc(i.SRC_ALPHA,i.ONE);break;case No:i.blendFuncSeparate(i.ZERO,i.ONE_MINUS_SRC_COLOR,i.ZERO,i.ONE);break;case Oo:i.blendFunc(i.ZERO,i.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",_);break}p=null,T=null,A=null,B=null,D.set(0,0,0),P=0,x=_,X=We}return}se=se||ne,Ee=Ee||Z,Ue=Ue||ie,(ne!==m||se!==S)&&(i.blendEquationSeparate(L[ne],L[se]),m=ne,S=se),(Z!==p||ie!==T||Ee!==A||Ue!==B)&&(i.blendFuncSeparate(xe[Z],xe[ie],xe[Ee],xe[Ue]),p=Z,T=ie,A=Ee,B=Ue),(et.equals(D)===!1||it!==P)&&(i.blendColor(et.r,et.g,et.b,it),D.copy(et),P=it),x=_,X=!1}function Xe(_,ne){_.side===Nt?q(i.CULL_FACE):j(i.CULL_FACE);let Z=_.side===yt;ne&&(Z=!Z),Me(Z),_.blending===oi&&_.transparent===!1?Te(fn):Te(_.blending,_.blendEquation,_.blendSrc,_.blendDst,_.blendEquationAlpha,_.blendSrcAlpha,_.blendDstAlpha,_.blendColor,_.blendAlpha,_.premultipliedAlpha),s.setFunc(_.depthFunc),s.setTest(_.depthTest),s.setMask(_.depthWrite),r.setMask(_.colorWrite);const ie=_.stencilWrite;o.setTest(ie),ie&&(o.setMask(_.stencilWriteMask),o.setFunc(_.stencilFunc,_.stencilRef,_.stencilFuncMask),o.setOp(_.stencilFail,_.stencilZFail,_.stencilZPass)),Ae(_.polygonOffset,_.polygonOffsetFactor,_.polygonOffsetUnits),_.alphaToCoverage===!0?j(i.SAMPLE_ALPHA_TO_COVERAGE):q(i.SAMPLE_ALPHA_TO_COVERAGE)}function Me(_){E!==_&&(_?i.frontFace(i.CW):i.frontFace(i.CCW),E=_)}function Be(_){_!==$h?(j(i.CULL_FACE),_!==M&&(_===Do?i.cullFace(i.BACK):_===Kh?i.cullFace(i.FRONT):i.cullFace(i.FRONT_AND_BACK))):q(i.CULL_FACE),M=_}function Ie(_){_!==C&&(V&&i.lineWidth(_),C=_)}function Ae(_,ne,Z){_?(j(i.POLYGON_OFFSET_FILL),(H!==ne||O!==Z)&&(i.polygonOffset(ne,Z),H=ne,O=Z)):q(i.POLYGON_OFFSET_FILL)}function Ve(_){_?j(i.SCISSOR_TEST):q(i.SCISSOR_TEST)}function R(_){_===void 0&&(_=i.TEXTURE0+$-1),v!==_&&(i.activeTexture(_),v=_)}function y(_,ne,Z){Z===void 0&&(v===null?Z=i.TEXTURE0+$-1:Z=v);let ie=w[Z];ie===void 0&&(ie={type:void 0,texture:void 0},w[Z]=ie),(ie.type!==_||ie.texture!==ne)&&(v!==Z&&(i.activeTexture(Z),v=Z),i.bindTexture(_,ne||k[_]),ie.type=_,ie.texture=ne)}function ee(){const _=w[v];_!==void 0&&_.type!==void 0&&(i.bindTexture(_.type,null),_.type=void 0,_.texture=void 0)}function re(){try{i.compressedTexImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function ae(){try{i.compressedTexImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function oe(){try{i.texSubImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function be(){try{i.texSubImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function de(){try{i.compressedTexSubImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function he(){try{i.compressedTexSubImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function Ne(){try{i.texStorage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function ce(){try{i.texStorage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function ye(){try{i.texImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function Ge(){try{i.texImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function Re(_){G.equals(_)===!1&&(i.scissor(_.x,_.y,_.z,_.w),G.copy(_))}function pe(_){K.equals(_)===!1&&(i.viewport(_.x,_.y,_.z,_.w),K.copy(_))}function Oe(_,ne){let Z=l.get(ne);Z===void 0&&(Z=new WeakMap,l.set(ne,Z));let ie=Z.get(_);ie===void 0&&(ie=i.getUniformBlockIndex(ne,_.name),Z.set(_,ie))}function Fe(_,ne){const ie=l.get(ne).get(_);a.get(ne)!==ie&&(i.uniformBlockBinding(ne,ie,_.__bindingPointIndex),a.set(ne,ie))}function Qe(){i.disable(i.BLEND),i.disable(i.CULL_FACE),i.disable(i.DEPTH_TEST),i.disable(i.POLYGON_OFFSET_FILL),i.disable(i.SCISSOR_TEST),i.disable(i.STENCIL_TEST),i.disable(i.SAMPLE_ALPHA_TO_COVERAGE),i.blendEquation(i.FUNC_ADD),i.blendFunc(i.ONE,i.ZERO),i.blendFuncSeparate(i.ONE,i.ZERO,i.ONE,i.ZERO),i.blendColor(0,0,0,0),i.colorMask(!0,!0,!0,!0),i.clearColor(0,0,0,0),i.depthMask(!0),i.depthFunc(i.LESS),i.clearDepth(1),i.stencilMask(4294967295),i.stencilFunc(i.ALWAYS,0,4294967295),i.stencilOp(i.KEEP,i.KEEP,i.KEEP),i.clearStencil(0),i.cullFace(i.BACK),i.frontFace(i.CCW),i.polygonOffset(0,0),i.activeTexture(i.TEXTURE0),i.bindFramebuffer(i.FRAMEBUFFER,null),i.bindFramebuffer(i.DRAW_FRAMEBUFFER,null),i.bindFramebuffer(i.READ_FRAMEBUFFER,null),i.useProgram(null),i.lineWidth(1),i.scissor(0,0,i.canvas.width,i.canvas.height),i.viewport(0,0,i.canvas.width,i.canvas.height),c={},v=null,w={},u={},h=new WeakMap,d=[],f=null,g=!1,x=null,m=null,p=null,T=null,S=null,A=null,B=null,D=new He(0,0,0),P=0,X=!1,E=null,M=null,C=null,H=null,O=null,G.set(0,0,i.canvas.width,i.canvas.height),K.set(0,0,i.canvas.width,i.canvas.height),r.reset(),s.reset(),o.reset()}return{buffers:{color:r,depth:s,stencil:o},enable:j,disable:q,bindFramebuffer:le,drawBuffers:ve,useProgram:ge,setBlending:Te,setMaterial:Xe,setFlipSided:Me,setCullFace:Be,setLineWidth:Ie,setPolygonOffset:Ae,setScissorTest:Ve,activeTexture:R,bindTexture:y,unbindTexture:ee,compressedTexImage2D:re,compressedTexImage3D:ae,texImage2D:ye,texImage3D:Ge,updateUBOMapping:Oe,uniformBlockBinding:Fe,texStorage2D:Ne,texStorage3D:ce,texSubImage2D:oe,texSubImage3D:be,compressedTexSubImage2D:de,compressedTexSubImage3D:he,scissor:Re,viewport:pe,reset:Qe}}function Q_(i,e,t,n,r,s,o){const a=e.has("WEBGL_multisampled_render_to_texture")?e.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),c=new De,u=new WeakMap;let h;const d=new WeakMap;let f=!1;try{f=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(R,y){return f?new OffscreenCanvas(R,y):yr("canvas")}function x(R,y,ee){let re=1;const ae=Ve(R);if((ae.width>ee||ae.height>ee)&&(re=ee/Math.max(ae.width,ae.height)),re<1)if(typeof HTMLImageElement<"u"&&R instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&R instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&R instanceof ImageBitmap||typeof VideoFrame<"u"&&R instanceof VideoFrame){const oe=Math.floor(re*ae.width),be=Math.floor(re*ae.height);h===void 0&&(h=g(oe,be));const de=y?g(oe,be):h;return de.width=oe,de.height=be,de.getContext("2d").drawImage(R,0,0,oe,be),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+ae.width+"x"+ae.height+") to ("+oe+"x"+be+")."),de}else return"data"in R&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+ae.width+"x"+ae.height+")."),R;return R}function m(R){return R.generateMipmaps&&R.minFilter!==Pt&&R.minFilter!==Ot}function p(R){i.generateMipmap(R)}function T(R,y,ee,re,ae=!1){if(R!==null){if(i[R]!==void 0)return i[R];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+R+"'")}let oe=y;if(y===i.RED&&(ee===i.FLOAT&&(oe=i.R32F),ee===i.HALF_FLOAT&&(oe=i.R16F),ee===i.UNSIGNED_BYTE&&(oe=i.R8)),y===i.RED_INTEGER&&(ee===i.UNSIGNED_BYTE&&(oe=i.R8UI),ee===i.UNSIGNED_SHORT&&(oe=i.R16UI),ee===i.UNSIGNED_INT&&(oe=i.R32UI),ee===i.BYTE&&(oe=i.R8I),ee===i.SHORT&&(oe=i.R16I),ee===i.INT&&(oe=i.R32I)),y===i.RG&&(ee===i.FLOAT&&(oe=i.RG32F),ee===i.HALF_FLOAT&&(oe=i.RG16F),ee===i.UNSIGNED_BYTE&&(oe=i.RG8)),y===i.RG_INTEGER&&(ee===i.UNSIGNED_BYTE&&(oe=i.RG8UI),ee===i.UNSIGNED_SHORT&&(oe=i.RG16UI),ee===i.UNSIGNED_INT&&(oe=i.RG32UI),ee===i.BYTE&&(oe=i.RG8I),ee===i.SHORT&&(oe=i.RG16I),ee===i.INT&&(oe=i.RG32I)),y===i.RGB&&ee===i.UNSIGNED_INT_5_9_9_9_REV&&(oe=i.RGB9_E5),y===i.RGBA){const be=ae?gr:je.getTransfer(re);ee===i.FLOAT&&(oe=i.RGBA32F),ee===i.HALF_FLOAT&&(oe=i.RGBA16F),ee===i.UNSIGNED_BYTE&&(oe=be===Ze?i.SRGB8_ALPHA8:i.RGBA8),ee===i.UNSIGNED_SHORT_4_4_4_4&&(oe=i.RGBA4),ee===i.UNSIGNED_SHORT_5_5_5_1&&(oe=i.RGB5_A1)}return(oe===i.R16F||oe===i.R32F||oe===i.RG16F||oe===i.RG32F||oe===i.RGBA16F||oe===i.RGBA32F)&&e.get("EXT_color_buffer_float"),oe}function S(R,y){let ee;return R?y===null||y===di||y===fi?ee=i.DEPTH24_STENCIL8:y===hn?ee=i.DEPTH32F_STENCIL8:y===mr&&(ee=i.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):y===null||y===di||y===fi?ee=i.DEPTH_COMPONENT24:y===hn?ee=i.DEPTH_COMPONENT32F:y===mr&&(ee=i.DEPTH_COMPONENT16),ee}function A(R,y){return m(R)===!0||R.isFramebufferTexture&&R.minFilter!==Pt&&R.minFilter!==Ot?Math.log2(Math.max(y.width,y.height))+1:R.mipmaps!==void 0&&R.mipmaps.length>0?R.mipmaps.length:R.isCompressedTexture&&Array.isArray(R.image)?y.mipmaps.length:1}function B(R){const y=R.target;y.removeEventListener("dispose",B),P(y),y.isVideoTexture&&u.delete(y)}function D(R){const y=R.target;y.removeEventListener("dispose",D),E(y)}function P(R){const y=n.get(R);if(y.__webglInit===void 0)return;const ee=R.source,re=d.get(ee);if(re){const ae=re[y.__cacheKey];ae.usedTimes--,ae.usedTimes===0&&X(R),Object.keys(re).length===0&&d.delete(ee)}n.remove(R)}function X(R){const y=n.get(R);i.deleteTexture(y.__webglTexture);const ee=R.source,re=d.get(ee);delete re[y.__cacheKey],o.memory.textures--}function E(R){const y=n.get(R);if(R.depthTexture&&R.depthTexture.dispose(),R.isWebGLCubeRenderTarget)for(let re=0;re<6;re++){if(Array.isArray(y.__webglFramebuffer[re]))for(let ae=0;ae<y.__webglFramebuffer[re].length;ae++)i.deleteFramebuffer(y.__webglFramebuffer[re][ae]);else i.deleteFramebuffer(y.__webglFramebuffer[re]);y.__webglDepthbuffer&&i.deleteRenderbuffer(y.__webglDepthbuffer[re])}else{if(Array.isArray(y.__webglFramebuffer))for(let re=0;re<y.__webglFramebuffer.length;re++)i.deleteFramebuffer(y.__webglFramebuffer[re]);else i.deleteFramebuffer(y.__webglFramebuffer);if(y.__webglDepthbuffer&&i.deleteRenderbuffer(y.__webglDepthbuffer),y.__webglMultisampledFramebuffer&&i.deleteFramebuffer(y.__webglMultisampledFramebuffer),y.__webglColorRenderbuffer)for(let re=0;re<y.__webglColorRenderbuffer.length;re++)y.__webglColorRenderbuffer[re]&&i.deleteRenderbuffer(y.__webglColorRenderbuffer[re]);y.__webglDepthRenderbuffer&&i.deleteRenderbuffer(y.__webglDepthRenderbuffer)}const ee=R.textures;for(let re=0,ae=ee.length;re<ae;re++){const oe=n.get(ee[re]);oe.__webglTexture&&(i.deleteTexture(oe.__webglTexture),o.memory.textures--),n.remove(ee[re])}n.remove(R)}let M=0;function C(){M=0}function H(){const R=M;return R>=r.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+R+" texture units while this GPU supports only "+r.maxTextures),M+=1,R}function O(R){const y=[];return y.push(R.wrapS),y.push(R.wrapT),y.push(R.wrapR||0),y.push(R.magFilter),y.push(R.minFilter),y.push(R.anisotropy),y.push(R.internalFormat),y.push(R.format),y.push(R.type),y.push(R.generateMipmaps),y.push(R.premultiplyAlpha),y.push(R.flipY),y.push(R.unpackAlignment),y.push(R.colorSpace),y.join()}function $(R,y){const ee=n.get(R);if(R.isVideoTexture&&Ie(R),R.isRenderTargetTexture===!1&&R.version>0&&ee.__version!==R.version){const re=R.image;if(re===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(re.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{K(ee,R,y);return}}t.bindTexture(i.TEXTURE_2D,ee.__webglTexture,i.TEXTURE0+y)}function V(R,y){const ee=n.get(R);if(R.version>0&&ee.__version!==R.version){K(ee,R,y);return}t.bindTexture(i.TEXTURE_2D_ARRAY,ee.__webglTexture,i.TEXTURE0+y)}function Y(R,y){const ee=n.get(R);if(R.version>0&&ee.__version!==R.version){K(ee,R,y);return}t.bindTexture(i.TEXTURE_3D,ee.__webglTexture,i.TEXTURE0+y)}function te(R,y){const ee=n.get(R);if(R.version>0&&ee.__version!==R.version){I(ee,R,y);return}t.bindTexture(i.TEXTURE_CUBE_MAP,ee.__webglTexture,i.TEXTURE0+y)}const v={[ma]:i.REPEAT,[Ln]:i.CLAMP_TO_EDGE,[ga]:i.MIRRORED_REPEAT},w={[Pt]:i.NEAREST,[Ud]:i.NEAREST_MIPMAP_NEAREST,[Fi]:i.NEAREST_MIPMAP_LINEAR,[Ot]:i.LINEAR,[Ts]:i.LINEAR_MIPMAP_NEAREST,[Un]:i.LINEAR_MIPMAP_LINEAR},U={[qd]:i.NEVER,[Jd]:i.ALWAYS,[Yd]:i.LESS,[qc]:i.LEQUAL,[jd]:i.EQUAL,[Zd]:i.GEQUAL,[$d]:i.GREATER,[Kd]:i.NOTEQUAL};function N(R,y){if(y.type===hn&&e.has("OES_texture_float_linear")===!1&&(y.magFilter===Ot||y.magFilter===Ts||y.magFilter===Fi||y.magFilter===Un||y.minFilter===Ot||y.minFilter===Ts||y.minFilter===Fi||y.minFilter===Un)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),i.texParameteri(R,i.TEXTURE_WRAP_S,v[y.wrapS]),i.texParameteri(R,i.TEXTURE_WRAP_T,v[y.wrapT]),(R===i.TEXTURE_3D||R===i.TEXTURE_2D_ARRAY)&&i.texParameteri(R,i.TEXTURE_WRAP_R,v[y.wrapR]),i.texParameteri(R,i.TEXTURE_MAG_FILTER,w[y.magFilter]),i.texParameteri(R,i.TEXTURE_MIN_FILTER,w[y.minFilter]),y.compareFunction&&(i.texParameteri(R,i.TEXTURE_COMPARE_MODE,i.COMPARE_REF_TO_TEXTURE),i.texParameteri(R,i.TEXTURE_COMPARE_FUNC,U[y.compareFunction])),e.has("EXT_texture_filter_anisotropic")===!0){if(y.magFilter===Pt||y.minFilter!==Fi&&y.minFilter!==Un||y.type===hn&&e.has("OES_texture_float_linear")===!1)return;if(y.anisotropy>1||n.get(y).__currentAnisotropy){const ee=e.get("EXT_texture_filter_anisotropic");i.texParameterf(R,ee.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(y.anisotropy,r.getMaxAnisotropy())),n.get(y).__currentAnisotropy=y.anisotropy}}}function G(R,y){let ee=!1;R.__webglInit===void 0&&(R.__webglInit=!0,y.addEventListener("dispose",B));const re=y.source;let ae=d.get(re);ae===void 0&&(ae={},d.set(re,ae));const oe=O(y);if(oe!==R.__cacheKey){ae[oe]===void 0&&(ae[oe]={texture:i.createTexture(),usedTimes:0},o.memory.textures++,ee=!0),ae[oe].usedTimes++;const be=ae[R.__cacheKey];be!==void 0&&(ae[R.__cacheKey].usedTimes--,be.usedTimes===0&&X(y)),R.__cacheKey=oe,R.__webglTexture=ae[oe].texture}return ee}function K(R,y,ee){let re=i.TEXTURE_2D;(y.isDataArrayTexture||y.isCompressedArrayTexture)&&(re=i.TEXTURE_2D_ARRAY),y.isData3DTexture&&(re=i.TEXTURE_3D);const ae=G(R,y),oe=y.source;t.bindTexture(re,R.__webglTexture,i.TEXTURE0+ee);const be=n.get(oe);if(oe.version!==be.__version||ae===!0){t.activeTexture(i.TEXTURE0+ee);const de=je.getPrimaries(je.workingColorSpace),he=y.colorSpace===un?null:je.getPrimaries(y.colorSpace),Ne=y.colorSpace===un||de===he?i.NONE:i.BROWSER_DEFAULT_WEBGL;i.pixelStorei(i.UNPACK_FLIP_Y_WEBGL,y.flipY),i.pixelStorei(i.UNPACK_PREMULTIPLY_ALPHA_WEBGL,y.premultiplyAlpha),i.pixelStorei(i.UNPACK_ALIGNMENT,y.unpackAlignment),i.pixelStorei(i.UNPACK_COLORSPACE_CONVERSION_WEBGL,Ne);let ce=x(y.image,!1,r.maxTextureSize);ce=Ae(y,ce);const ye=s.convert(y.format,y.colorSpace),Ge=s.convert(y.type);let Re=T(y.internalFormat,ye,Ge,y.colorSpace,y.isVideoTexture);N(re,y);let pe;const Oe=y.mipmaps,Fe=y.isVideoTexture!==!0,Qe=be.__version===void 0||ae===!0,_=oe.dataReady,ne=A(y,ce);if(y.isDepthTexture)Re=S(y.format===pi,y.type),Qe&&(Fe?t.texStorage2D(i.TEXTURE_2D,1,Re,ce.width,ce.height):t.texImage2D(i.TEXTURE_2D,0,Re,ce.width,ce.height,0,ye,Ge,null));else if(y.isDataTexture)if(Oe.length>0){Fe&&Qe&&t.texStorage2D(i.TEXTURE_2D,ne,Re,Oe[0].width,Oe[0].height);for(let Z=0,ie=Oe.length;Z<ie;Z++)pe=Oe[Z],Fe?_&&t.texSubImage2D(i.TEXTURE_2D,Z,0,0,pe.width,pe.height,ye,Ge,pe.data):t.texImage2D(i.TEXTURE_2D,Z,Re,pe.width,pe.height,0,ye,Ge,pe.data);y.generateMipmaps=!1}else Fe?(Qe&&t.texStorage2D(i.TEXTURE_2D,ne,Re,ce.width,ce.height),_&&t.texSubImage2D(i.TEXTURE_2D,0,0,0,ce.width,ce.height,ye,Ge,ce.data)):t.texImage2D(i.TEXTURE_2D,0,Re,ce.width,ce.height,0,ye,Ge,ce.data);else if(y.isCompressedTexture)if(y.isCompressedArrayTexture){Fe&&Qe&&t.texStorage3D(i.TEXTURE_2D_ARRAY,ne,Re,Oe[0].width,Oe[0].height,ce.depth);for(let Z=0,ie=Oe.length;Z<ie;Z++)if(pe=Oe[Z],y.format!==Vt)if(ye!==null)if(Fe){if(_)if(y.layerUpdates.size>0){for(const se of y.layerUpdates){const Ee=pe.width*pe.height;t.compressedTexSubImage3D(i.TEXTURE_2D_ARRAY,Z,0,0,se,pe.width,pe.height,1,ye,pe.data.slice(Ee*se,Ee*(se+1)),0,0)}y.clearLayerUpdates()}else t.compressedTexSubImage3D(i.TEXTURE_2D_ARRAY,Z,0,0,0,pe.width,pe.height,ce.depth,ye,pe.data,0,0)}else t.compressedTexImage3D(i.TEXTURE_2D_ARRAY,Z,Re,pe.width,pe.height,ce.depth,0,pe.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else Fe?_&&t.texSubImage3D(i.TEXTURE_2D_ARRAY,Z,0,0,0,pe.width,pe.height,ce.depth,ye,Ge,pe.data):t.texImage3D(i.TEXTURE_2D_ARRAY,Z,Re,pe.width,pe.height,ce.depth,0,ye,Ge,pe.data)}else{Fe&&Qe&&t.texStorage2D(i.TEXTURE_2D,ne,Re,Oe[0].width,Oe[0].height);for(let Z=0,ie=Oe.length;Z<ie;Z++)pe=Oe[Z],y.format!==Vt?ye!==null?Fe?_&&t.compressedTexSubImage2D(i.TEXTURE_2D,Z,0,0,pe.width,pe.height,ye,pe.data):t.compressedTexImage2D(i.TEXTURE_2D,Z,Re,pe.width,pe.height,0,pe.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):Fe?_&&t.texSubImage2D(i.TEXTURE_2D,Z,0,0,pe.width,pe.height,ye,Ge,pe.data):t.texImage2D(i.TEXTURE_2D,Z,Re,pe.width,pe.height,0,ye,Ge,pe.data)}else if(y.isDataArrayTexture)if(Fe){if(Qe&&t.texStorage3D(i.TEXTURE_2D_ARRAY,ne,Re,ce.width,ce.height,ce.depth),_)if(y.layerUpdates.size>0){let Z;switch(Ge){case i.UNSIGNED_BYTE:switch(ye){case i.ALPHA:Z=1;break;case i.LUMINANCE:Z=1;break;case i.LUMINANCE_ALPHA:Z=2;break;case i.RGB:Z=3;break;case i.RGBA:Z=4;break;default:throw new Error(`Unknown texel size for format ${ye}.`)}break;case i.UNSIGNED_SHORT_4_4_4_4:case i.UNSIGNED_SHORT_5_5_5_1:case i.UNSIGNED_SHORT_5_6_5:Z=1;break;default:throw new Error(`Unknown texel size for type ${Ge}.`)}const ie=ce.width*ce.height*Z;for(const se of y.layerUpdates)t.texSubImage3D(i.TEXTURE_2D_ARRAY,0,0,0,se,ce.width,ce.height,1,ye,Ge,ce.data.slice(ie*se,ie*(se+1)));y.clearLayerUpdates()}else t.texSubImage3D(i.TEXTURE_2D_ARRAY,0,0,0,0,ce.width,ce.height,ce.depth,ye,Ge,ce.data)}else t.texImage3D(i.TEXTURE_2D_ARRAY,0,Re,ce.width,ce.height,ce.depth,0,ye,Ge,ce.data);else if(y.isData3DTexture)Fe?(Qe&&t.texStorage3D(i.TEXTURE_3D,ne,Re,ce.width,ce.height,ce.depth),_&&t.texSubImage3D(i.TEXTURE_3D,0,0,0,0,ce.width,ce.height,ce.depth,ye,Ge,ce.data)):t.texImage3D(i.TEXTURE_3D,0,Re,ce.width,ce.height,ce.depth,0,ye,Ge,ce.data);else if(y.isFramebufferTexture){if(Qe)if(Fe)t.texStorage2D(i.TEXTURE_2D,ne,Re,ce.width,ce.height);else{let Z=ce.width,ie=ce.height;for(let se=0;se<ne;se++)t.texImage2D(i.TEXTURE_2D,se,Re,Z,ie,0,ye,Ge,null),Z>>=1,ie>>=1}}else if(Oe.length>0){if(Fe&&Qe){const Z=Ve(Oe[0]);t.texStorage2D(i.TEXTURE_2D,ne,Re,Z.width,Z.height)}for(let Z=0,ie=Oe.length;Z<ie;Z++)pe=Oe[Z],Fe?_&&t.texSubImage2D(i.TEXTURE_2D,Z,0,0,ye,Ge,pe):t.texImage2D(i.TEXTURE_2D,Z,Re,ye,Ge,pe);y.generateMipmaps=!1}else if(Fe){if(Qe){const Z=Ve(ce);t.texStorage2D(i.TEXTURE_2D,ne,Re,Z.width,Z.height)}_&&t.texSubImage2D(i.TEXTURE_2D,0,0,0,ye,Ge,ce)}else t.texImage2D(i.TEXTURE_2D,0,Re,ye,Ge,ce);m(y)&&p(re),be.__version=oe.version,y.onUpdate&&y.onUpdate(y)}R.__version=y.version}function I(R,y,ee){if(y.image.length!==6)return;const re=G(R,y),ae=y.source;t.bindTexture(i.TEXTURE_CUBE_MAP,R.__webglTexture,i.TEXTURE0+ee);const oe=n.get(ae);if(ae.version!==oe.__version||re===!0){t.activeTexture(i.TEXTURE0+ee);const be=je.getPrimaries(je.workingColorSpace),de=y.colorSpace===un?null:je.getPrimaries(y.colorSpace),he=y.colorSpace===un||be===de?i.NONE:i.BROWSER_DEFAULT_WEBGL;i.pixelStorei(i.UNPACK_FLIP_Y_WEBGL,y.flipY),i.pixelStorei(i.UNPACK_PREMULTIPLY_ALPHA_WEBGL,y.premultiplyAlpha),i.pixelStorei(i.UNPACK_ALIGNMENT,y.unpackAlignment),i.pixelStorei(i.UNPACK_COLORSPACE_CONVERSION_WEBGL,he);const Ne=y.isCompressedTexture||y.image[0].isCompressedTexture,ce=y.image[0]&&y.image[0].isDataTexture,ye=[];for(let ie=0;ie<6;ie++)!Ne&&!ce?ye[ie]=x(y.image[ie],!0,r.maxCubemapSize):ye[ie]=ce?y.image[ie].image:y.image[ie],ye[ie]=Ae(y,ye[ie]);const Ge=ye[0],Re=s.convert(y.format,y.colorSpace),pe=s.convert(y.type),Oe=T(y.internalFormat,Re,pe,y.colorSpace),Fe=y.isVideoTexture!==!0,Qe=oe.__version===void 0||re===!0,_=ae.dataReady;let ne=A(y,Ge);N(i.TEXTURE_CUBE_MAP,y);let Z;if(Ne){Fe&&Qe&&t.texStorage2D(i.TEXTURE_CUBE_MAP,ne,Oe,Ge.width,Ge.height);for(let ie=0;ie<6;ie++){Z=ye[ie].mipmaps;for(let se=0;se<Z.length;se++){const Ee=Z[se];y.format!==Vt?Re!==null?Fe?_&&t.compressedTexSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,0,0,Ee.width,Ee.height,Re,Ee.data):t.compressedTexImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,Oe,Ee.width,Ee.height,0,Ee.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,0,0,Ee.width,Ee.height,Re,pe,Ee.data):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,Oe,Ee.width,Ee.height,0,Re,pe,Ee.data)}}}else{if(Z=y.mipmaps,Fe&&Qe){Z.length>0&&ne++;const ie=Ve(ye[0]);t.texStorage2D(i.TEXTURE_CUBE_MAP,ne,Oe,ie.width,ie.height)}for(let ie=0;ie<6;ie++)if(ce){Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,0,0,ye[ie].width,ye[ie].height,Re,pe,ye[ie].data):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,Oe,ye[ie].width,ye[ie].height,0,Re,pe,ye[ie].data);for(let se=0;se<Z.length;se++){const Ue=Z[se].image[ie].image;Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,0,0,Ue.width,Ue.height,Re,pe,Ue.data):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,Oe,Ue.width,Ue.height,0,Re,pe,Ue.data)}}else{Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,0,0,Re,pe,ye[ie]):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,Oe,Re,pe,ye[ie]);for(let se=0;se<Z.length;se++){const Ee=Z[se];Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,0,0,Re,pe,Ee.image[ie]):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,Oe,Re,pe,Ee.image[ie])}}}m(y)&&p(i.TEXTURE_CUBE_MAP),oe.__version=ae.version,y.onUpdate&&y.onUpdate(y)}R.__version=y.version}function k(R,y,ee,re,ae,oe){const be=s.convert(ee.format,ee.colorSpace),de=s.convert(ee.type),he=T(ee.internalFormat,be,de,ee.colorSpace);if(!n.get(y).__hasExternalTextures){const ce=Math.max(1,y.width>>oe),ye=Math.max(1,y.height>>oe);ae===i.TEXTURE_3D||ae===i.TEXTURE_2D_ARRAY?t.texImage3D(ae,oe,he,ce,ye,y.depth,0,be,de,null):t.texImage2D(ae,oe,he,ce,ye,0,be,de,null)}t.bindFramebuffer(i.FRAMEBUFFER,R),Be(y)?a.framebufferTexture2DMultisampleEXT(i.FRAMEBUFFER,re,ae,n.get(ee).__webglTexture,0,Me(y)):(ae===i.TEXTURE_2D||ae>=i.TEXTURE_CUBE_MAP_POSITIVE_X&&ae<=i.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&i.framebufferTexture2D(i.FRAMEBUFFER,re,ae,n.get(ee).__webglTexture,oe),t.bindFramebuffer(i.FRAMEBUFFER,null)}function j(R,y,ee){if(i.bindRenderbuffer(i.RENDERBUFFER,R),y.depthBuffer){const re=y.depthTexture,ae=re&&re.isDepthTexture?re.type:null,oe=S(y.stencilBuffer,ae),be=y.stencilBuffer?i.DEPTH_STENCIL_ATTACHMENT:i.DEPTH_ATTACHMENT,de=Me(y);Be(y)?a.renderbufferStorageMultisampleEXT(i.RENDERBUFFER,de,oe,y.width,y.height):ee?i.renderbufferStorageMultisample(i.RENDERBUFFER,de,oe,y.width,y.height):i.renderbufferStorage(i.RENDERBUFFER,oe,y.width,y.height),i.framebufferRenderbuffer(i.FRAMEBUFFER,be,i.RENDERBUFFER,R)}else{const re=y.textures;for(let ae=0;ae<re.length;ae++){const oe=re[ae],be=s.convert(oe.format,oe.colorSpace),de=s.convert(oe.type),he=T(oe.internalFormat,be,de,oe.colorSpace),Ne=Me(y);ee&&Be(y)===!1?i.renderbufferStorageMultisample(i.RENDERBUFFER,Ne,he,y.width,y.height):Be(y)?a.renderbufferStorageMultisampleEXT(i.RENDERBUFFER,Ne,he,y.width,y.height):i.renderbufferStorage(i.RENDERBUFFER,he,y.width,y.height)}}i.bindRenderbuffer(i.RENDERBUFFER,null)}function q(R,y){if(y&&y.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(t.bindFramebuffer(i.FRAMEBUFFER,R),!(y.depthTexture&&y.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!n.get(y.depthTexture).__webglTexture||y.depthTexture.image.width!==y.width||y.depthTexture.image.height!==y.height)&&(y.depthTexture.image.width=y.width,y.depthTexture.image.height=y.height,y.depthTexture.needsUpdate=!0),$(y.depthTexture,0);const re=n.get(y.depthTexture).__webglTexture,ae=Me(y);if(y.depthTexture.format===li)Be(y)?a.framebufferTexture2DMultisampleEXT(i.FRAMEBUFFER,i.DEPTH_ATTACHMENT,i.TEXTURE_2D,re,0,ae):i.framebufferTexture2D(i.FRAMEBUFFER,i.DEPTH_ATTACHMENT,i.TEXTURE_2D,re,0);else if(y.depthTexture.format===pi)Be(y)?a.framebufferTexture2DMultisampleEXT(i.FRAMEBUFFER,i.DEPTH_STENCIL_ATTACHMENT,i.TEXTURE_2D,re,0,ae):i.framebufferTexture2D(i.FRAMEBUFFER,i.DEPTH_STENCIL_ATTACHMENT,i.TEXTURE_2D,re,0);else throw new Error("Unknown depthTexture format")}function le(R){const y=n.get(R),ee=R.isWebGLCubeRenderTarget===!0;if(R.depthTexture&&!y.__autoAllocateDepthBuffer){if(ee)throw new Error("target.depthTexture not supported in Cube render targets");q(y.__webglFramebuffer,R)}else if(ee){y.__webglDepthbuffer=[];for(let re=0;re<6;re++)t.bindFramebuffer(i.FRAMEBUFFER,y.__webglFramebuffer[re]),y.__webglDepthbuffer[re]=i.createRenderbuffer(),j(y.__webglDepthbuffer[re],R,!1)}else t.bindFramebuffer(i.FRAMEBUFFER,y.__webglFramebuffer),y.__webglDepthbuffer=i.createRenderbuffer(),j(y.__webglDepthbuffer,R,!1);t.bindFramebuffer(i.FRAMEBUFFER,null)}function ve(R,y,ee){const re=n.get(R);y!==void 0&&k(re.__webglFramebuffer,R,R.texture,i.COLOR_ATTACHMENT0,i.TEXTURE_2D,0),ee!==void 0&&le(R)}function ge(R){const y=R.texture,ee=n.get(R),re=n.get(y);R.addEventListener("dispose",D);const ae=R.textures,oe=R.isWebGLCubeRenderTarget===!0,be=ae.length>1;if(be||(re.__webglTexture===void 0&&(re.__webglTexture=i.createTexture()),re.__version=y.version,o.memory.textures++),oe){ee.__webglFramebuffer=[];for(let de=0;de<6;de++)if(y.mipmaps&&y.mipmaps.length>0){ee.__webglFramebuffer[de]=[];for(let he=0;he<y.mipmaps.length;he++)ee.__webglFramebuffer[de][he]=i.createFramebuffer()}else ee.__webglFramebuffer[de]=i.createFramebuffer()}else{if(y.mipmaps&&y.mipmaps.length>0){ee.__webglFramebuffer=[];for(let de=0;de<y.mipmaps.length;de++)ee.__webglFramebuffer[de]=i.createFramebuffer()}else ee.__webglFramebuffer=i.createFramebuffer();if(be)for(let de=0,he=ae.length;de<he;de++){const Ne=n.get(ae[de]);Ne.__webglTexture===void 0&&(Ne.__webglTexture=i.createTexture(),o.memory.textures++)}if(R.samples>0&&Be(R)===!1){ee.__webglMultisampledFramebuffer=i.createFramebuffer(),ee.__webglColorRenderbuffer=[],t.bindFramebuffer(i.FRAMEBUFFER,ee.__webglMultisampledFramebuffer);for(let de=0;de<ae.length;de++){const he=ae[de];ee.__webglColorRenderbuffer[de]=i.createRenderbuffer(),i.bindRenderbuffer(i.RENDERBUFFER,ee.__webglColorRenderbuffer[de]);const Ne=s.convert(he.format,he.colorSpace),ce=s.convert(he.type),ye=T(he.internalFormat,Ne,ce,he.colorSpace,R.isXRRenderTarget===!0),Ge=Me(R);i.renderbufferStorageMultisample(i.RENDERBUFFER,Ge,ye,R.width,R.height),i.framebufferRenderbuffer(i.FRAMEBUFFER,i.COLOR_ATTACHMENT0+de,i.RENDERBUFFER,ee.__webglColorRenderbuffer[de])}i.bindRenderbuffer(i.RENDERBUFFER,null),R.depthBuffer&&(ee.__webglDepthRenderbuffer=i.createRenderbuffer(),j(ee.__webglDepthRenderbuffer,R,!0)),t.bindFramebuffer(i.FRAMEBUFFER,null)}}if(oe){t.bindTexture(i.TEXTURE_CUBE_MAP,re.__webglTexture),N(i.TEXTURE_CUBE_MAP,y);for(let de=0;de<6;de++)if(y.mipmaps&&y.mipmaps.length>0)for(let he=0;he<y.mipmaps.length;he++)k(ee.__webglFramebuffer[de][he],R,y,i.COLOR_ATTACHMENT0,i.TEXTURE_CUBE_MAP_POSITIVE_X+de,he);else k(ee.__webglFramebuffer[de],R,y,i.COLOR_ATTACHMENT0,i.TEXTURE_CUBE_MAP_POSITIVE_X+de,0);m(y)&&p(i.TEXTURE_CUBE_MAP),t.unbindTexture()}else if(be){for(let de=0,he=ae.length;de<he;de++){const Ne=ae[de],ce=n.get(Ne);t.bindTexture(i.TEXTURE_2D,ce.__webglTexture),N(i.TEXTURE_2D,Ne),k(ee.__webglFramebuffer,R,Ne,i.COLOR_ATTACHMENT0+de,i.TEXTURE_2D,0),m(Ne)&&p(i.TEXTURE_2D)}t.unbindTexture()}else{let de=i.TEXTURE_2D;if((R.isWebGL3DRenderTarget||R.isWebGLArrayRenderTarget)&&(de=R.isWebGL3DRenderTarget?i.TEXTURE_3D:i.TEXTURE_2D_ARRAY),t.bindTexture(de,re.__webglTexture),N(de,y),y.mipmaps&&y.mipmaps.length>0)for(let he=0;he<y.mipmaps.length;he++)k(ee.__webglFramebuffer[he],R,y,i.COLOR_ATTACHMENT0,de,he);else k(ee.__webglFramebuffer,R,y,i.COLOR_ATTACHMENT0,de,0);m(y)&&p(de),t.unbindTexture()}R.depthBuffer&&le(R)}function L(R){const y=R.textures;for(let ee=0,re=y.length;ee<re;ee++){const ae=y[ee];if(m(ae)){const oe=R.isWebGLCubeRenderTarget?i.TEXTURE_CUBE_MAP:i.TEXTURE_2D,be=n.get(ae).__webglTexture;t.bindTexture(oe,be),p(oe),t.unbindTexture()}}}const xe=[],Te=[];function Xe(R){if(R.samples>0){if(Be(R)===!1){const y=R.textures,ee=R.width,re=R.height;let ae=i.COLOR_BUFFER_BIT;const oe=R.stencilBuffer?i.DEPTH_STENCIL_ATTACHMENT:i.DEPTH_ATTACHMENT,be=n.get(R),de=y.length>1;if(de)for(let he=0;he<y.length;he++)t.bindFramebuffer(i.FRAMEBUFFER,be.__webglMultisampledFramebuffer),i.framebufferRenderbuffer(i.FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.RENDERBUFFER,null),t.bindFramebuffer(i.FRAMEBUFFER,be.__webglFramebuffer),i.framebufferTexture2D(i.DRAW_FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.TEXTURE_2D,null,0);t.bindFramebuffer(i.READ_FRAMEBUFFER,be.__webglMultisampledFramebuffer),t.bindFramebuffer(i.DRAW_FRAMEBUFFER,be.__webglFramebuffer);for(let he=0;he<y.length;he++){if(R.resolveDepthBuffer&&(R.depthBuffer&&(ae|=i.DEPTH_BUFFER_BIT),R.stencilBuffer&&R.resolveStencilBuffer&&(ae|=i.STENCIL_BUFFER_BIT)),de){i.framebufferRenderbuffer(i.READ_FRAMEBUFFER,i.COLOR_ATTACHMENT0,i.RENDERBUFFER,be.__webglColorRenderbuffer[he]);const Ne=n.get(y[he]).__webglTexture;i.framebufferTexture2D(i.DRAW_FRAMEBUFFER,i.COLOR_ATTACHMENT0,i.TEXTURE_2D,Ne,0)}i.blitFramebuffer(0,0,ee,re,0,0,ee,re,ae,i.NEAREST),l===!0&&(xe.length=0,Te.length=0,xe.push(i.COLOR_ATTACHMENT0+he),R.depthBuffer&&R.resolveDepthBuffer===!1&&(xe.push(oe),Te.push(oe),i.invalidateFramebuffer(i.DRAW_FRAMEBUFFER,Te)),i.invalidateFramebuffer(i.READ_FRAMEBUFFER,xe))}if(t.bindFramebuffer(i.READ_FRAMEBUFFER,null),t.bindFramebuffer(i.DRAW_FRAMEBUFFER,null),de)for(let he=0;he<y.length;he++){t.bindFramebuffer(i.FRAMEBUFFER,be.__webglMultisampledFramebuffer),i.framebufferRenderbuffer(i.FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.RENDERBUFFER,be.__webglColorRenderbuffer[he]);const Ne=n.get(y[he]).__webglTexture;t.bindFramebuffer(i.FRAMEBUFFER,be.__webglFramebuffer),i.framebufferTexture2D(i.DRAW_FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.TEXTURE_2D,Ne,0)}t.bindFramebuffer(i.DRAW_FRAMEBUFFER,be.__webglMultisampledFramebuffer)}else if(R.depthBuffer&&R.resolveDepthBuffer===!1&&l){const y=R.stencilBuffer?i.DEPTH_STENCIL_ATTACHMENT:i.DEPTH_ATTACHMENT;i.invalidateFramebuffer(i.DRAW_FRAMEBUFFER,[y])}}}function Me(R){return Math.min(r.maxSamples,R.samples)}function Be(R){const y=n.get(R);return R.samples>0&&e.has("WEBGL_multisampled_render_to_texture")===!0&&y.__useRenderToTexture!==!1}function Ie(R){const y=o.render.frame;u.get(R)!==y&&(u.set(R,y),R.update())}function Ae(R,y){const ee=R.colorSpace,re=R.format,ae=R.type;return R.isCompressedTexture===!0||R.isVideoTexture===!0||ee!==vn&&ee!==un&&(je.getTransfer(ee)===Ze?(re!==Vt||ae!==gn)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",ee)),y}function Ve(R){return typeof HTMLImageElement<"u"&&R instanceof HTMLImageElement?(c.width=R.naturalWidth||R.width,c.height=R.naturalHeight||R.height):typeof VideoFrame<"u"&&R instanceof VideoFrame?(c.width=R.displayWidth,c.height=R.displayHeight):(c.width=R.width,c.height=R.height),c}this.allocateTextureUnit=H,this.resetTextureUnits=C,this.setTexture2D=$,this.setTexture2DArray=V,this.setTexture3D=Y,this.setTextureCube=te,this.rebindTextures=ve,this.setupRenderTarget=ge,this.updateRenderTargetMipmap=L,this.updateMultisampleRenderTarget=Xe,this.setupDepthRenderbuffer=le,this.setupFrameBufferTexture=k,this.useMultisampledRTT=Be}function ev(i,e){function t(n,r=un){let s;const o=je.getTransfer(r);if(n===gn)return i.UNSIGNED_BYTE;if(n===zc)return i.UNSIGNED_SHORT_4_4_4_4;if(n===Vc)return i.UNSIGNED_SHORT_5_5_5_1;if(n===Nd)return i.UNSIGNED_INT_5_9_9_9_REV;if(n===Dd)return i.BYTE;if(n===Id)return i.SHORT;if(n===mr)return i.UNSIGNED_SHORT;if(n===kc)return i.INT;if(n===di)return i.UNSIGNED_INT;if(n===hn)return i.FLOAT;if(n===Tr)return i.HALF_FLOAT;if(n===Od)return i.ALPHA;if(n===Fd)return i.RGB;if(n===Vt)return i.RGBA;if(n===Bd)return i.LUMINANCE;if(n===kd)return i.LUMINANCE_ALPHA;if(n===li)return i.DEPTH_COMPONENT;if(n===pi)return i.DEPTH_STENCIL;if(n===zd)return i.RED;if(n===Gc)return i.RED_INTEGER;if(n===Vd)return i.RG;if(n===Hc)return i.RG_INTEGER;if(n===Wc)return i.RGBA_INTEGER;if(n===ws||n===As||n===Cs||n===Rs)if(o===Ze)if(s=e.get("WEBGL_compressed_texture_s3tc_srgb"),s!==null){if(n===ws)return s.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(n===As)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(n===Cs)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(n===Rs)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(s=e.get("WEBGL_compressed_texture_s3tc"),s!==null){if(n===ws)return s.COMPRESSED_RGB_S3TC_DXT1_EXT;if(n===As)return s.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(n===Cs)return s.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(n===Rs)return s.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(n===Fo||n===Bo||n===ko||n===zo)if(s=e.get("WEBGL_compressed_texture_pvrtc"),s!==null){if(n===Fo)return s.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(n===Bo)return s.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(n===ko)return s.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(n===zo)return s.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(n===Vo||n===Go||n===Ho)if(s=e.get("WEBGL_compressed_texture_etc"),s!==null){if(n===Vo||n===Go)return o===Ze?s.COMPRESSED_SRGB8_ETC2:s.COMPRESSED_RGB8_ETC2;if(n===Ho)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:s.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(n===Wo||n===Xo||n===qo||n===Yo||n===jo||n===$o||n===Ko||n===Zo||n===Jo||n===Qo||n===el||n===tl||n===nl||n===il)if(s=e.get("WEBGL_compressed_texture_astc"),s!==null){if(n===Wo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:s.COMPRESSED_RGBA_ASTC_4x4_KHR;if(n===Xo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:s.COMPRESSED_RGBA_ASTC_5x4_KHR;if(n===qo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:s.COMPRESSED_RGBA_ASTC_5x5_KHR;if(n===Yo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:s.COMPRESSED_RGBA_ASTC_6x5_KHR;if(n===jo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:s.COMPRESSED_RGBA_ASTC_6x6_KHR;if(n===$o)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:s.COMPRESSED_RGBA_ASTC_8x5_KHR;if(n===Ko)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:s.COMPRESSED_RGBA_ASTC_8x6_KHR;if(n===Zo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:s.COMPRESSED_RGBA_ASTC_8x8_KHR;if(n===Jo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:s.COMPRESSED_RGBA_ASTC_10x5_KHR;if(n===Qo)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:s.COMPRESSED_RGBA_ASTC_10x6_KHR;if(n===el)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:s.COMPRESSED_RGBA_ASTC_10x8_KHR;if(n===tl)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:s.COMPRESSED_RGBA_ASTC_10x10_KHR;if(n===nl)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:s.COMPRESSED_RGBA_ASTC_12x10_KHR;if(n===il)return o===Ze?s.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:s.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(n===Ps||n===rl||n===sl)if(s=e.get("EXT_texture_compression_bptc"),s!==null){if(n===Ps)return o===Ze?s.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:s.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(n===rl)return s.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(n===sl)return s.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(n===Gd||n===al||n===ol||n===ll)if(s=e.get("EXT_texture_compression_rgtc"),s!==null){if(n===Ps)return s.COMPRESSED_RED_RGTC1_EXT;if(n===al)return s.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(n===ol)return s.COMPRESSED_RED_GREEN_RGTC2_EXT;if(n===ll)return s.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return n===fi?i.UNSIGNED_INT_24_8:i[n]!==void 0?i[n]:null}return{convert:t}}class tv extends Rt{constructor(e=[]){super(),this.isArrayCamera=!0,this.cameras=e}}class dn extends dt{constructor(){super(),this.isGroup=!0,this.type="Group"}}const nv={type:"move"};class na{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new dn,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new dn,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new z,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new z),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new dn,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new z,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new z),this._grip}dispatchEvent(e){return this._targetRay!==null&&this._targetRay.dispatchEvent(e),this._grip!==null&&this._grip.dispatchEvent(e),this._hand!==null&&this._hand.dispatchEvent(e),this}connect(e){if(e&&e.hand){const t=this._hand;if(t)for(const n of e.hand.values())this._getHandJoint(t,n)}return this.dispatchEvent({type:"connected",data:e}),this}disconnect(e){return this.dispatchEvent({type:"disconnected",data:e}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(e,t,n){let r=null,s=null,o=null;const a=this._targetRay,l=this._grip,c=this._hand;if(e&&t.session.visibilityState!=="visible-blurred"){if(c&&e.hand){o=!0;for(const x of e.hand.values()){const m=t.getJointPose(x,n),p=this._getHandJoint(c,x);m!==null&&(p.matrix.fromArray(m.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=m.radius),p.visible=m!==null}const u=c.joints["index-finger-tip"],h=c.joints["thumb-tip"],d=u.position.distanceTo(h.position),f=.02,g=.005;c.inputState.pinching&&d>f+g?(c.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:e.handedness,target:this})):!c.inputState.pinching&&d<=f-g&&(c.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:e.handedness,target:this}))}else l!==null&&e.gripSpace&&(s=t.getPose(e.gripSpace,n),s!==null&&(l.matrix.fromArray(s.transform.matrix),l.matrix.decompose(l.position,l.rotation,l.scale),l.matrixWorldNeedsUpdate=!0,s.linearVelocity?(l.hasLinearVelocity=!0,l.linearVelocity.copy(s.linearVelocity)):l.hasLinearVelocity=!1,s.angularVelocity?(l.hasAngularVelocity=!0,l.angularVelocity.copy(s.angularVelocity)):l.hasAngularVelocity=!1));a!==null&&(r=t.getPose(e.targetRaySpace,n),r===null&&s!==null&&(r=s),r!==null&&(a.matrix.fromArray(r.transform.matrix),a.matrix.decompose(a.position,a.rotation,a.scale),a.matrixWorldNeedsUpdate=!0,r.linearVelocity?(a.hasLinearVelocity=!0,a.linearVelocity.copy(r.linearVelocity)):a.hasLinearVelocity=!1,r.angularVelocity?(a.hasAngularVelocity=!0,a.angularVelocity.copy(r.angularVelocity)):a.hasAngularVelocity=!1,this.dispatchEvent(nv)))}return a!==null&&(a.visible=r!==null),l!==null&&(l.visible=s!==null),c!==null&&(c.visible=o!==null),this}_getHandJoint(e,t){if(e.joints[t.jointName]===void 0){const n=new dn;n.matrixAutoUpdate=!1,n.visible=!1,e.joints[t.jointName]=n,e.add(n)}return e.joints[t.jointName]}}const iv=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,rv=`
uniform sampler2DArray depthColor;
uniform float depthWidth;
uniform float depthHeight;

void main() {

	vec2 coord = vec2( gl_FragCoord.x / depthWidth, gl_FragCoord.y / depthHeight );

	if ( coord.x >= 1.0 ) {

		gl_FragDepth = texture( depthColor, vec3( coord.x - 1.0, coord.y, 1 ) ).r;

	} else {

		gl_FragDepth = texture( depthColor, vec3( coord.x, coord.y, 0 ) ).r;

	}

}`;class sv{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(e,t,n){if(this.texture===null){const r=new St,s=e.properties.get(r);s.__webglTexture=t.texture,(t.depthNear!=n.depthNear||t.depthFar!=n.depthFar)&&(this.depthNear=t.depthNear,this.depthFar=t.depthFar),this.texture=r}}getMesh(e){if(this.texture!==null&&this.mesh===null){const t=e.cameras[0].viewport,n=new _n({vertexShader:iv,fragmentShader:rv,uniforms:{depthColor:{value:this.texture},depthWidth:{value:t.z},depthHeight:{value:t.w}}});this.mesh=new Ft(new Cr(20,20),n)}return this.mesh}reset(){this.texture=null,this.mesh=null}}class av extends Bn{constructor(e,t){super();const n=this;let r=null,s=1,o=null,a="local-floor",l=1,c=null,u=null,h=null,d=null,f=null,g=null;const x=new sv,m=t.getContextAttributes();let p=null,T=null;const S=[],A=[],B=new De;let D=null;const P=new Rt;P.layers.enable(1),P.viewport=new ht;const X=new Rt;X.layers.enable(2),X.viewport=new ht;const E=[P,X],M=new tv;M.layers.enable(1),M.layers.enable(2);let C=null,H=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(I){let k=S[I];return k===void 0&&(k=new na,S[I]=k),k.getTargetRaySpace()},this.getControllerGrip=function(I){let k=S[I];return k===void 0&&(k=new na,S[I]=k),k.getGripSpace()},this.getHand=function(I){let k=S[I];return k===void 0&&(k=new na,S[I]=k),k.getHandSpace()};function O(I){const k=A.indexOf(I.inputSource);if(k===-1)return;const j=S[k];j!==void 0&&(j.update(I.inputSource,I.frame,c||o),j.dispatchEvent({type:I.type,data:I.inputSource}))}function $(){r.removeEventListener("select",O),r.removeEventListener("selectstart",O),r.removeEventListener("selectend",O),r.removeEventListener("squeeze",O),r.removeEventListener("squeezestart",O),r.removeEventListener("squeezeend",O),r.removeEventListener("end",$),r.removeEventListener("inputsourceschange",V);for(let I=0;I<S.length;I++){const k=A[I];k!==null&&(A[I]=null,S[I].disconnect(k))}C=null,H=null,x.reset(),e.setRenderTarget(p),f=null,d=null,h=null,r=null,T=null,K.stop(),n.isPresenting=!1,e.setPixelRatio(D),e.setSize(B.width,B.height,!1),n.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(I){s=I,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(I){a=I,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return c||o},this.setReferenceSpace=function(I){c=I},this.getBaseLayer=function(){return d!==null?d:f},this.getBinding=function(){return h},this.getFrame=function(){return g},this.getSession=function(){return r},this.setSession=async function(I){if(r=I,r!==null){if(p=e.getRenderTarget(),r.addEventListener("select",O),r.addEventListener("selectstart",O),r.addEventListener("selectend",O),r.addEventListener("squeeze",O),r.addEventListener("squeezestart",O),r.addEventListener("squeezeend",O),r.addEventListener("end",$),r.addEventListener("inputsourceschange",V),m.xrCompatible!==!0&&await t.makeXRCompatible(),D=e.getPixelRatio(),e.getSize(B),r.renderState.layers===void 0){const k={antialias:m.antialias,alpha:!0,depth:m.depth,stencil:m.stencil,framebufferScaleFactor:s};f=new XRWebGLLayer(r,t,k),r.updateRenderState({baseLayer:f}),e.setPixelRatio(1),e.setSize(f.framebufferWidth,f.framebufferHeight,!1),T=new Dn(f.framebufferWidth,f.framebufferHeight,{format:Vt,type:gn,colorSpace:e.outputColorSpace,stencilBuffer:m.stencil})}else{let k=null,j=null,q=null;m.depth&&(q=m.stencil?t.DEPTH24_STENCIL8:t.DEPTH_COMPONENT24,k=m.stencil?pi:li,j=m.stencil?fi:di);const le={colorFormat:t.RGBA8,depthFormat:q,scaleFactor:s};h=new XRWebGLBinding(r,t),d=h.createProjectionLayer(le),r.updateRenderState({layers:[d]}),e.setPixelRatio(1),e.setSize(d.textureWidth,d.textureHeight,!1),T=new Dn(d.textureWidth,d.textureHeight,{format:Vt,type:gn,depthTexture:new ou(d.textureWidth,d.textureHeight,j,void 0,void 0,void 0,void 0,void 0,void 0,k),stencilBuffer:m.stencil,colorSpace:e.outputColorSpace,samples:m.antialias?4:0,resolveDepthBuffer:d.ignoreDepthValues===!1})}T.isXRRenderTarget=!0,this.setFoveation(l),c=null,o=await r.requestReferenceSpace(a),K.setContext(r),K.start(),n.isPresenting=!0,n.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(r!==null)return r.environmentBlendMode};function V(I){for(let k=0;k<I.removed.length;k++){const j=I.removed[k],q=A.indexOf(j);q>=0&&(A[q]=null,S[q].disconnect(j))}for(let k=0;k<I.added.length;k++){const j=I.added[k];let q=A.indexOf(j);if(q===-1){for(let ve=0;ve<S.length;ve++)if(ve>=A.length){A.push(j),q=ve;break}else if(A[ve]===null){A[ve]=j,q=ve;break}if(q===-1)break}const le=S[q];le&&le.connect(j)}}const Y=new z,te=new z;function v(I,k,j){Y.setFromMatrixPosition(k.matrixWorld),te.setFromMatrixPosition(j.matrixWorld);const q=Y.distanceTo(te),le=k.projectionMatrix.elements,ve=j.projectionMatrix.elements,ge=le[14]/(le[10]-1),L=le[14]/(le[10]+1),xe=(le[9]+1)/le[5],Te=(le[9]-1)/le[5],Xe=(le[8]-1)/le[0],Me=(ve[8]+1)/ve[0],Be=ge*Xe,Ie=ge*Me,Ae=q/(-Xe+Me),Ve=Ae*-Xe;k.matrixWorld.decompose(I.position,I.quaternion,I.scale),I.translateX(Ve),I.translateZ(Ae),I.matrixWorld.compose(I.position,I.quaternion,I.scale),I.matrixWorldInverse.copy(I.matrixWorld).invert();const R=ge+Ae,y=L+Ae,ee=Be-Ve,re=Ie+(q-Ve),ae=xe*L/y*R,oe=Te*L/y*R;I.projectionMatrix.makePerspective(ee,re,ae,oe,R,y),I.projectionMatrixInverse.copy(I.projectionMatrix).invert()}function w(I,k){k===null?I.matrixWorld.copy(I.matrix):I.matrixWorld.multiplyMatrices(k.matrixWorld,I.matrix),I.matrixWorldInverse.copy(I.matrixWorld).invert()}this.updateCamera=function(I){if(r===null)return;x.texture!==null&&(I.near=x.depthNear,I.far=x.depthFar),M.near=X.near=P.near=I.near,M.far=X.far=P.far=I.far,(C!==M.near||H!==M.far)&&(r.updateRenderState({depthNear:M.near,depthFar:M.far}),C=M.near,H=M.far,P.near=C,P.far=H,X.near=C,X.far=H,P.updateProjectionMatrix(),X.updateProjectionMatrix(),I.updateProjectionMatrix());const k=I.parent,j=M.cameras;w(M,k);for(let q=0;q<j.length;q++)w(j[q],k);j.length===2?v(M,P,X):M.projectionMatrix.copy(P.projectionMatrix),U(I,M,k)};function U(I,k,j){j===null?I.matrix.copy(k.matrixWorld):(I.matrix.copy(j.matrixWorld),I.matrix.invert(),I.matrix.multiply(k.matrixWorld)),I.matrix.decompose(I.position,I.quaternion,I.scale),I.updateMatrixWorld(!0),I.projectionMatrix.copy(k.projectionMatrix),I.projectionMatrixInverse.copy(k.projectionMatrixInverse),I.isPerspectiveCamera&&(I.fov=_a*2*Math.atan(1/I.projectionMatrix.elements[5]),I.zoom=1)}this.getCamera=function(){return M},this.getFoveation=function(){if(!(d===null&&f===null))return l},this.setFoveation=function(I){l=I,d!==null&&(d.fixedFoveation=I),f!==null&&f.fixedFoveation!==void 0&&(f.fixedFoveation=I)},this.hasDepthSensing=function(){return x.texture!==null},this.getDepthSensingMesh=function(){return x.getMesh(M)};let N=null;function G(I,k){if(u=k.getViewerPose(c||o),g=k,u!==null){const j=u.views;f!==null&&(e.setRenderTargetFramebuffer(T,f.framebuffer),e.setRenderTarget(T));let q=!1;j.length!==M.cameras.length&&(M.cameras.length=0,q=!0);for(let ve=0;ve<j.length;ve++){const ge=j[ve];let L=null;if(f!==null)L=f.getViewport(ge);else{const Te=h.getViewSubImage(d,ge);L=Te.viewport,ve===0&&(e.setRenderTargetTextures(T,Te.colorTexture,d.ignoreDepthValues?void 0:Te.depthStencilTexture),e.setRenderTarget(T))}let xe=E[ve];xe===void 0&&(xe=new Rt,xe.layers.enable(ve),xe.viewport=new ht,E[ve]=xe),xe.matrix.fromArray(ge.transform.matrix),xe.matrix.decompose(xe.position,xe.quaternion,xe.scale),xe.projectionMatrix.fromArray(ge.projectionMatrix),xe.projectionMatrixInverse.copy(xe.projectionMatrix).invert(),xe.viewport.set(L.x,L.y,L.width,L.height),ve===0&&(M.matrix.copy(xe.matrix),M.matrix.decompose(M.position,M.quaternion,M.scale)),q===!0&&M.cameras.push(xe)}const le=r.enabledFeatures;if(le&&le.includes("depth-sensing")){const ve=h.getDepthInformation(j[0]);ve&&ve.isValid&&ve.texture&&x.init(e,ve,r.renderState)}}for(let j=0;j<S.length;j++){const q=A[j],le=S[j];q!==null&&le!==void 0&&le.update(q,k,c||o)}N&&N(I,k),k.detectedPlanes&&n.dispatchEvent({type:"planesdetected",data:k}),g=null}const K=new su;K.setAnimationLoop(G),this.setAnimationLoop=function(I){N=I},this.dispose=function(){}}}const An=new Wt,ov=new Je;function lv(i,e){function t(m,p){m.matrixAutoUpdate===!0&&m.updateMatrix(),p.value.copy(m.matrix)}function n(m,p){p.color.getRGB(m.fogColor.value,nu(i)),p.isFog?(m.fogNear.value=p.near,m.fogFar.value=p.far):p.isFogExp2&&(m.fogDensity.value=p.density)}function r(m,p,T,S,A){p.isMeshBasicMaterial||p.isMeshLambertMaterial?s(m,p):p.isMeshToonMaterial?(s(m,p),h(m,p)):p.isMeshPhongMaterial?(s(m,p),u(m,p)):p.isMeshStandardMaterial?(s(m,p),d(m,p),p.isMeshPhysicalMaterial&&f(m,p,A)):p.isMeshMatcapMaterial?(s(m,p),g(m,p)):p.isMeshDepthMaterial?s(m,p):p.isMeshDistanceMaterial?(s(m,p),x(m,p)):p.isMeshNormalMaterial?s(m,p):p.isLineBasicMaterial?(o(m,p),p.isLineDashedMaterial&&a(m,p)):p.isPointsMaterial?l(m,p,T,S):p.isSpriteMaterial?c(m,p):p.isShadowMaterial?(m.color.value.copy(p.color),m.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function s(m,p){m.opacity.value=p.opacity,p.color&&m.diffuse.value.copy(p.color),p.emissive&&m.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(m.map.value=p.map,t(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.bumpMap&&(m.bumpMap.value=p.bumpMap,t(p.bumpMap,m.bumpMapTransform),m.bumpScale.value=p.bumpScale,p.side===yt&&(m.bumpScale.value*=-1)),p.normalMap&&(m.normalMap.value=p.normalMap,t(p.normalMap,m.normalMapTransform),m.normalScale.value.copy(p.normalScale),p.side===yt&&m.normalScale.value.negate()),p.displacementMap&&(m.displacementMap.value=p.displacementMap,t(p.displacementMap,m.displacementMapTransform),m.displacementScale.value=p.displacementScale,m.displacementBias.value=p.displacementBias),p.emissiveMap&&(m.emissiveMap.value=p.emissiveMap,t(p.emissiveMap,m.emissiveMapTransform)),p.specularMap&&(m.specularMap.value=p.specularMap,t(p.specularMap,m.specularMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest);const T=e.get(p),S=T.envMap,A=T.envMapRotation;S&&(m.envMap.value=S,An.copy(A),An.x*=-1,An.y*=-1,An.z*=-1,S.isCubeTexture&&S.isRenderTargetTexture===!1&&(An.y*=-1,An.z*=-1),m.envMapRotation.value.setFromMatrix4(ov.makeRotationFromEuler(An)),m.flipEnvMap.value=S.isCubeTexture&&S.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=p.reflectivity,m.ior.value=p.ior,m.refractionRatio.value=p.refractionRatio),p.lightMap&&(m.lightMap.value=p.lightMap,m.lightMapIntensity.value=p.lightMapIntensity,t(p.lightMap,m.lightMapTransform)),p.aoMap&&(m.aoMap.value=p.aoMap,m.aoMapIntensity.value=p.aoMapIntensity,t(p.aoMap,m.aoMapTransform))}function o(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,p.map&&(m.map.value=p.map,t(p.map,m.mapTransform))}function a(m,p){m.dashSize.value=p.dashSize,m.totalSize.value=p.dashSize+p.gapSize,m.scale.value=p.scale}function l(m,p,T,S){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.size.value=p.size*T,m.scale.value=S*.5,p.map&&(m.map.value=p.map,t(p.map,m.uvTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function c(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.rotation.value=p.rotation,p.map&&(m.map.value=p.map,t(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function u(m,p){m.specular.value.copy(p.specular),m.shininess.value=Math.max(p.shininess,1e-4)}function h(m,p){p.gradientMap&&(m.gradientMap.value=p.gradientMap)}function d(m,p){m.metalness.value=p.metalness,p.metalnessMap&&(m.metalnessMap.value=p.metalnessMap,t(p.metalnessMap,m.metalnessMapTransform)),m.roughness.value=p.roughness,p.roughnessMap&&(m.roughnessMap.value=p.roughnessMap,t(p.roughnessMap,m.roughnessMapTransform)),p.envMap&&(m.envMapIntensity.value=p.envMapIntensity)}function f(m,p,T){m.ior.value=p.ior,p.sheen>0&&(m.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),m.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(m.sheenColorMap.value=p.sheenColorMap,t(p.sheenColorMap,m.sheenColorMapTransform)),p.sheenRoughnessMap&&(m.sheenRoughnessMap.value=p.sheenRoughnessMap,t(p.sheenRoughnessMap,m.sheenRoughnessMapTransform))),p.clearcoat>0&&(m.clearcoat.value=p.clearcoat,m.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(m.clearcoatMap.value=p.clearcoatMap,t(p.clearcoatMap,m.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,t(p.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(m.clearcoatNormalMap.value=p.clearcoatNormalMap,t(p.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===yt&&m.clearcoatNormalScale.value.negate())),p.dispersion>0&&(m.dispersion.value=p.dispersion),p.iridescence>0&&(m.iridescence.value=p.iridescence,m.iridescenceIOR.value=p.iridescenceIOR,m.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(m.iridescenceMap.value=p.iridescenceMap,t(p.iridescenceMap,m.iridescenceMapTransform)),p.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=p.iridescenceThicknessMap,t(p.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),p.transmission>0&&(m.transmission.value=p.transmission,m.transmissionSamplerMap.value=T.texture,m.transmissionSamplerSize.value.set(T.width,T.height),p.transmissionMap&&(m.transmissionMap.value=p.transmissionMap,t(p.transmissionMap,m.transmissionMapTransform)),m.thickness.value=p.thickness,p.thicknessMap&&(m.thicknessMap.value=p.thicknessMap,t(p.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=p.attenuationDistance,m.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(m.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(m.anisotropyMap.value=p.anisotropyMap,t(p.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=p.specularIntensity,m.specularColor.value.copy(p.specularColor),p.specularColorMap&&(m.specularColorMap.value=p.specularColorMap,t(p.specularColorMap,m.specularColorMapTransform)),p.specularIntensityMap&&(m.specularIntensityMap.value=p.specularIntensityMap,t(p.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,p){p.matcap&&(m.matcap.value=p.matcap)}function x(m,p){const T=e.get(p).light;m.referencePosition.value.setFromMatrixPosition(T.matrixWorld),m.nearDistance.value=T.shadow.camera.near,m.farDistance.value=T.shadow.camera.far}return{refreshFogUniforms:n,refreshMaterialUniforms:r}}function cv(i,e,t,n){let r={},s={},o=[];const a=i.getParameter(i.MAX_UNIFORM_BUFFER_BINDINGS);function l(T,S){const A=S.program;n.uniformBlockBinding(T,A)}function c(T,S){let A=r[T.id];A===void 0&&(g(T),A=u(T),r[T.id]=A,T.addEventListener("dispose",m));const B=S.program;n.updateUBOMapping(T,B);const D=e.render.frame;s[T.id]!==D&&(d(T),s[T.id]=D)}function u(T){const S=h();T.__bindingPointIndex=S;const A=i.createBuffer(),B=T.__size,D=T.usage;return i.bindBuffer(i.UNIFORM_BUFFER,A),i.bufferData(i.UNIFORM_BUFFER,B,D),i.bindBuffer(i.UNIFORM_BUFFER,null),i.bindBufferBase(i.UNIFORM_BUFFER,S,A),A}function h(){for(let T=0;T<a;T++)if(o.indexOf(T)===-1)return o.push(T),T;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(T){const S=r[T.id],A=T.uniforms,B=T.__cache;i.bindBuffer(i.UNIFORM_BUFFER,S);for(let D=0,P=A.length;D<P;D++){const X=Array.isArray(A[D])?A[D]:[A[D]];for(let E=0,M=X.length;E<M;E++){const C=X[E];if(f(C,D,E,B)===!0){const H=C.__offset,O=Array.isArray(C.value)?C.value:[C.value];let $=0;for(let V=0;V<O.length;V++){const Y=O[V],te=x(Y);typeof Y=="number"||typeof Y=="boolean"?(C.__data[0]=Y,i.bufferSubData(i.UNIFORM_BUFFER,H+$,C.__data)):Y.isMatrix3?(C.__data[0]=Y.elements[0],C.__data[1]=Y.elements[1],C.__data[2]=Y.elements[2],C.__data[3]=0,C.__data[4]=Y.elements[3],C.__data[5]=Y.elements[4],C.__data[6]=Y.elements[5],C.__data[7]=0,C.__data[8]=Y.elements[6],C.__data[9]=Y.elements[7],C.__data[10]=Y.elements[8],C.__data[11]=0):(Y.toArray(C.__data,$),$+=te.storage/Float32Array.BYTES_PER_ELEMENT)}i.bufferSubData(i.UNIFORM_BUFFER,H,C.__data)}}}i.bindBuffer(i.UNIFORM_BUFFER,null)}function f(T,S,A,B){const D=T.value,P=S+"_"+A;if(B[P]===void 0)return typeof D=="number"||typeof D=="boolean"?B[P]=D:B[P]=D.clone(),!0;{const X=B[P];if(typeof D=="number"||typeof D=="boolean"){if(X!==D)return B[P]=D,!0}else if(X.equals(D)===!1)return X.copy(D),!0}return!1}function g(T){const S=T.uniforms;let A=0;const B=16;for(let P=0,X=S.length;P<X;P++){const E=Array.isArray(S[P])?S[P]:[S[P]];for(let M=0,C=E.length;M<C;M++){const H=E[M],O=Array.isArray(H.value)?H.value:[H.value];for(let $=0,V=O.length;$<V;$++){const Y=O[$],te=x(Y),v=A%B;v!==0&&B-v<te.boundary&&(A+=B-v),H.__data=new Float32Array(te.storage/Float32Array.BYTES_PER_ELEMENT),H.__offset=A,A+=te.storage}}}const D=A%B;return D>0&&(A+=B-D),T.__size=A,T.__cache={},this}function x(T){const S={boundary:0,storage:0};return typeof T=="number"||typeof T=="boolean"?(S.boundary=4,S.storage=4):T.isVector2?(S.boundary=8,S.storage=8):T.isVector3||T.isColor?(S.boundary=16,S.storage=12):T.isVector4?(S.boundary=16,S.storage=16):T.isMatrix3?(S.boundary=48,S.storage=48):T.isMatrix4?(S.boundary=64,S.storage=64):T.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",T),S}function m(T){const S=T.target;S.removeEventListener("dispose",m);const A=o.indexOf(S.__bindingPointIndex);o.splice(A,1),i.deleteBuffer(r[S.id]),delete r[S.id],delete s[S.id]}function p(){for(const T in r)i.deleteBuffer(r[T]);o=[],r={},s={}}return{bind:l,update:c,dispose:p}}class uv{constructor(e={}){const{canvas:t=tf(),context:n=null,depth:r=!0,stencil:s=!1,alpha:o=!1,antialias:a=!1,premultipliedAlpha:l=!0,preserveDrawingBuffer:c=!1,powerPreference:u="default",failIfMajorPerformanceCaveat:h=!1}=e;this.isWebGLRenderer=!0;let d;if(n!==null){if(typeof WebGLRenderingContext<"u"&&n instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");d=n.getContextAttributes().alpha}else d=o;const f=new Uint32Array(4),g=new Int32Array(4);let x=null,m=null;const p=[],T=[];this.domElement=t,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=Bt,this.toneMapping=pn,this.toneMappingExposure=1;const S=this;let A=!1,B=0,D=0,P=null,X=-1,E=null;const M=new ht,C=new ht;let H=null;const O=new He(0);let $=0,V=t.width,Y=t.height,te=1,v=null,w=null;const U=new ht(0,0,V,Y),N=new ht(0,0,V,Y);let G=!1;const K=new Aa;let I=!1,k=!1;const j=new Je,q=new z,le={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};let ve=!1;function ge(){return P===null?te:1}let L=n;function xe(b,F){return t.getContext(b,F)}try{const b={alpha:!0,depth:r,stencil:s,antialias:a,premultipliedAlpha:l,preserveDrawingBuffer:c,powerPreference:u,failIfMajorPerformanceCaveat:h};if("setAttribute"in t&&t.setAttribute("data-engine",`three.js r${Ta}`),t.addEventListener("webglcontextlost",ne,!1),t.addEventListener("webglcontextrestored",Z,!1),t.addEventListener("webglcontextcreationerror",ie,!1),L===null){const F="webgl2";if(L=xe(F,b),L===null)throw xe(F)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(b){throw console.error("THREE.WebGLRenderer: "+b.message),b}let Te,Xe,Me,Be,Ie,Ae,Ve,R,y,ee,re,ae,oe,be,de,he,Ne,ce,ye,Ge,Re,pe,Oe,Fe;function Qe(){Te=new vg(L),Te.init(),pe=new ev(L,Te),Xe=new dg(L,Te,e,pe),Me=new J_(L),Be=new Sg(L),Ie=new B_,Ae=new Q_(L,Te,Me,Ie,Xe,pe,Be),Ve=new pg(S),R=new _g(S),y=new Cf(L),Oe=new ug(L,y),ee=new xg(L,y,Be,Oe),re=new bg(L,ee,y,Be),ye=new Mg(L,Xe,Ae),he=new fg(Ie),ae=new F_(S,Ve,R,Te,Xe,Oe,he),oe=new lv(S,Ie),be=new z_,de=new q_(Te),ce=new cg(S,Ve,R,Me,re,d,l),Ne=new Z_(S,re,Xe),Fe=new cv(L,Be,Xe,Me),Ge=new hg(L,Te,Be),Re=new yg(L,Te,Be),Be.programs=ae.programs,S.capabilities=Xe,S.extensions=Te,S.properties=Ie,S.renderLists=be,S.shadowMap=Ne,S.state=Me,S.info=Be}Qe();const _=new av(S,L);this.xr=_,this.getContext=function(){return L},this.getContextAttributes=function(){return L.getContextAttributes()},this.forceContextLoss=function(){const b=Te.get("WEBGL_lose_context");b&&b.loseContext()},this.forceContextRestore=function(){const b=Te.get("WEBGL_lose_context");b&&b.restoreContext()},this.getPixelRatio=function(){return te},this.setPixelRatio=function(b){b!==void 0&&(te=b,this.setSize(V,Y,!1))},this.getSize=function(b){return b.set(V,Y)},this.setSize=function(b,F,J=!0){if(_.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}V=b,Y=F,t.width=Math.floor(b*te),t.height=Math.floor(F*te),J===!0&&(t.style.width=b+"px",t.style.height=F+"px"),this.setViewport(0,0,b,F)},this.getDrawingBufferSize=function(b){return b.set(V*te,Y*te).floor()},this.setDrawingBufferSize=function(b,F,J){V=b,Y=F,te=J,t.width=Math.floor(b*J),t.height=Math.floor(F*J),this.setViewport(0,0,b,F)},this.getCurrentViewport=function(b){return b.copy(M)},this.getViewport=function(b){return b.copy(U)},this.setViewport=function(b,F,J,Q){b.isVector4?U.set(b.x,b.y,b.z,b.w):U.set(b,F,J,Q),Me.viewport(M.copy(U).multiplyScalar(te).round())},this.getScissor=function(b){return b.copy(N)},this.setScissor=function(b,F,J,Q){b.isVector4?N.set(b.x,b.y,b.z,b.w):N.set(b,F,J,Q),Me.scissor(C.copy(N).multiplyScalar(te).round())},this.getScissorTest=function(){return G},this.setScissorTest=function(b){Me.setScissorTest(G=b)},this.setOpaqueSort=function(b){v=b},this.setTransparentSort=function(b){w=b},this.getClearColor=function(b){return b.copy(ce.getClearColor())},this.setClearColor=function(){ce.setClearColor.apply(ce,arguments)},this.getClearAlpha=function(){return ce.getClearAlpha()},this.setClearAlpha=function(){ce.setClearAlpha.apply(ce,arguments)},this.clear=function(b=!0,F=!0,J=!0){let Q=0;if(b){let W=!1;if(P!==null){const ue=P.texture.format;W=ue===Wc||ue===Hc||ue===Gc}if(W){const ue=P.texture.type,me=ue===gn||ue===di||ue===mr||ue===fi||ue===zc||ue===Vc,_e=ce.getClearColor(),Se=ce.getClearAlpha(),Pe=_e.r,Le=_e.g,Ce=_e.b;me?(f[0]=Pe,f[1]=Le,f[2]=Ce,f[3]=Se,L.clearBufferuiv(L.COLOR,0,f)):(g[0]=Pe,g[1]=Le,g[2]=Ce,g[3]=Se,L.clearBufferiv(L.COLOR,0,g))}else Q|=L.COLOR_BUFFER_BIT}F&&(Q|=L.DEPTH_BUFFER_BIT),J&&(Q|=L.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),L.clear(Q)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){t.removeEventListener("webglcontextlost",ne,!1),t.removeEventListener("webglcontextrestored",Z,!1),t.removeEventListener("webglcontextcreationerror",ie,!1),be.dispose(),de.dispose(),Ie.dispose(),Ve.dispose(),R.dispose(),re.dispose(),Oe.dispose(),Fe.dispose(),ae.dispose(),_.dispose(),_.removeEventListener("sessionstart",rt),_.removeEventListener("sessionend",st),Mt.stop()};function ne(b){b.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),A=!0}function Z(){console.log("THREE.WebGLRenderer: Context Restored."),A=!1;const b=Be.autoReset,F=Ne.enabled,J=Ne.autoUpdate,Q=Ne.needsUpdate,W=Ne.type;Qe(),Be.autoReset=b,Ne.enabled=F,Ne.autoUpdate=J,Ne.needsUpdate=Q,Ne.type=W}function ie(b){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",b.statusMessage)}function se(b){const F=b.target;F.removeEventListener("dispose",se),Ee(F)}function Ee(b){Ue(b),Ie.remove(b)}function Ue(b){const F=Ie.get(b).programs;F!==void 0&&(F.forEach(function(J){ae.releaseProgram(J)}),b.isShaderMaterial&&ae.releaseShaderCache(b))}this.renderBufferDirect=function(b,F,J,Q,W,ue){F===null&&(F=le);const me=W.isMesh&&W.matrixWorld.determinant()<0,_e=vu(b,F,J,Q,W);Me.setMaterial(Q,me);let Se=J.index,Pe=1;if(Q.wireframe===!0){if(Se=ee.getWireframeAttribute(J),Se===void 0)return;Pe=2}const Le=J.drawRange,Ce=J.attributes.position;let qe=Le.start*Pe,tt=(Le.start+Le.count)*Pe;ue!==null&&(qe=Math.max(qe,ue.start*Pe),tt=Math.min(tt,(ue.start+ue.count)*Pe)),Se!==null?(qe=Math.max(qe,0),tt=Math.min(tt,Se.count)):Ce!=null&&(qe=Math.max(qe,0),tt=Math.min(tt,Ce.count));const nt=tt-qe;if(nt<0||nt===1/0)return;Oe.setup(W,Q,_e,J,Se);let Et,Ye=Ge;if(Se!==null&&(Et=y.get(Se),Ye=Re,Ye.setIndex(Et)),W.isMesh)Q.wireframe===!0?(Me.setLineWidth(Q.wireframeLinewidth*ge()),Ye.setMode(L.LINES)):Ye.setMode(L.TRIANGLES);else if(W.isLine){let we=Q.linewidth;we===void 0&&(we=1),Me.setLineWidth(we*ge()),W.isLineSegments?Ye.setMode(L.LINES):W.isLineLoop?Ye.setMode(L.LINE_LOOP):Ye.setMode(L.LINE_STRIP)}else W.isPoints?Ye.setMode(L.POINTS):W.isSprite&&Ye.setMode(L.TRIANGLES);if(W.isBatchedMesh)W._multiDrawInstances!==null?Ye.renderMultiDrawInstances(W._multiDrawStarts,W._multiDrawCounts,W._multiDrawCount,W._multiDrawInstances):Ye.renderMultiDraw(W._multiDrawStarts,W._multiDrawCounts,W._multiDrawCount);else if(W.isInstancedMesh)Ye.renderInstances(qe,nt,W.count);else if(J.isInstancedBufferGeometry){const we=J._maxInstanceCount!==void 0?J._maxInstanceCount:1/0,gt=Math.min(J.instanceCount,we);Ye.renderInstances(qe,nt,gt)}else Ye.render(qe,nt)};function et(b,F,J){b.transparent===!0&&b.side===Nt&&b.forceSinglePass===!1?(b.side=yt,b.needsUpdate=!0,Ni(b,F,J),b.side=mn,b.needsUpdate=!0,Ni(b,F,J),b.side=Nt):Ni(b,F,J)}this.compile=function(b,F,J=null){J===null&&(J=b),m=de.get(J),m.init(F),T.push(m),J.traverseVisible(function(W){W.isLight&&W.layers.test(F.layers)&&(m.pushLight(W),W.castShadow&&m.pushShadow(W))}),b!==J&&b.traverseVisible(function(W){W.isLight&&W.layers.test(F.layers)&&(m.pushLight(W),W.castShadow&&m.pushShadow(W))}),m.setupLights();const Q=new Set;return b.traverse(function(W){const ue=W.material;if(ue)if(Array.isArray(ue))for(let me=0;me<ue.length;me++){const _e=ue[me];et(_e,J,W),Q.add(_e)}else et(ue,J,W),Q.add(ue)}),T.pop(),m=null,Q},this.compileAsync=function(b,F,J=null){const Q=this.compile(b,F,J);return new Promise(W=>{function ue(){if(Q.forEach(function(me){Ie.get(me).currentProgram.isReady()&&Q.delete(me)}),Q.size===0){W(b);return}setTimeout(ue,10)}Te.get("KHR_parallel_shader_compile")!==null?ue():setTimeout(ue,10)})};let it=null;function We(b){it&&it(b)}function rt(){Mt.stop()}function st(){Mt.start()}const Mt=new su;Mt.setAnimationLoop(We),typeof self<"u"&&Mt.setContext(self),this.setAnimationLoop=function(b){it=b,_.setAnimationLoop(b),b===null?Mt.stop():Mt.start()},_.addEventListener("sessionstart",rt),_.addEventListener("sessionend",st),this.render=function(b,F){if(F!==void 0&&F.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(A===!0)return;if(b.matrixWorldAutoUpdate===!0&&b.updateMatrixWorld(),F.parent===null&&F.matrixWorldAutoUpdate===!0&&F.updateMatrixWorld(),_.enabled===!0&&_.isPresenting===!0&&(_.cameraAutoUpdate===!0&&_.updateCamera(F),F=_.getCamera()),b.isScene===!0&&b.onBeforeRender(S,b,F,P),m=de.get(b,T.length),m.init(F),T.push(m),j.multiplyMatrices(F.projectionMatrix,F.matrixWorldInverse),K.setFromProjectionMatrix(j),k=this.localClippingEnabled,I=he.init(this.clippingPlanes,k),x=be.get(b,p.length),x.init(),p.push(x),_.enabled===!0&&_.isPresenting===!0){const ue=S.xr.getDepthSensingMesh();ue!==null&&bt(ue,F,-1/0,S.sortObjects)}bt(b,F,0,S.sortObjects),x.finish(),S.sortObjects===!0&&x.sort(v,w),ve=_.enabled===!1||_.isPresenting===!1||_.hasDepthSensing()===!1,ve&&ce.addToRenderList(x,b),this.info.render.frame++,I===!0&&he.beginShadows();const J=m.state.shadowsArray;Ne.render(J,b,F),I===!0&&he.endShadows(),this.info.autoReset===!0&&this.info.reset();const Q=x.opaque,W=x.transmissive;if(m.setupLights(),F.isArrayCamera){const ue=F.cameras;if(W.length>0)for(let me=0,_e=ue.length;me<_e;me++){const Se=ue[me];xn(Q,W,b,Se)}ve&&ce.render(b);for(let me=0,_e=ue.length;me<_e;me++){const Se=ue[me];Qt(x,b,Se,Se.viewport)}}else W.length>0&&xn(Q,W,b,F),ve&&ce.render(b),Qt(x,b,F);P!==null&&(Ae.updateMultisampleRenderTarget(P),Ae.updateRenderTargetMipmap(P)),b.isScene===!0&&b.onAfterRender(S,b,F),Oe.resetDefaultState(),X=-1,E=null,T.pop(),T.length>0?(m=T[T.length-1],I===!0&&he.setGlobalState(S.clippingPlanes,m.state.camera)):m=null,p.pop(),p.length>0?x=p[p.length-1]:x=null};function bt(b,F,J,Q){if(b.visible===!1)return;if(b.layers.test(F.layers)){if(b.isGroup)J=b.renderOrder;else if(b.isLOD)b.autoUpdate===!0&&b.update(F);else if(b.isLight)m.pushLight(b),b.castShadow&&m.pushShadow(b);else if(b.isSprite){if(!b.frustumCulled||K.intersectsSprite(b)){Q&&q.setFromMatrixPosition(b.matrixWorld).applyMatrix4(j);const me=re.update(b),_e=b.material;_e.visible&&x.push(b,me,_e,J,q.z,null)}}else if((b.isMesh||b.isLine||b.isPoints)&&(!b.frustumCulled||K.intersectsObject(b))){const me=re.update(b),_e=b.material;if(Q&&(b.boundingSphere!==void 0?(b.boundingSphere===null&&b.computeBoundingSphere(),q.copy(b.boundingSphere.center)):(me.boundingSphere===null&&me.computeBoundingSphere(),q.copy(me.boundingSphere.center)),q.applyMatrix4(b.matrixWorld).applyMatrix4(j)),Array.isArray(_e)){const Se=me.groups;for(let Pe=0,Le=Se.length;Pe<Le;Pe++){const Ce=Se[Pe],qe=_e[Ce.materialIndex];qe&&qe.visible&&x.push(b,me,qe,J,q.z,Ce)}}else _e.visible&&x.push(b,me,_e,J,q.z,null)}}const ue=b.children;for(let me=0,_e=ue.length;me<_e;me++)bt(ue[me],F,J,Q)}function Qt(b,F,J,Q){const W=b.opaque,ue=b.transmissive,me=b.transparent;m.setupLightsView(J),I===!0&&he.setGlobalState(S.clippingPlanes,J),Q&&Me.viewport(M.copy(Q)),W.length>0&&yn(W,F,J),ue.length>0&&yn(ue,F,J),me.length>0&&yn(me,F,J),Me.buffers.depth.setTest(!0),Me.buffers.depth.setMask(!0),Me.buffers.color.setMask(!0),Me.setPolygonOffset(!1)}function xn(b,F,J,Q){if((J.isScene===!0?J.overrideMaterial:null)!==null)return;m.state.transmissionRenderTarget[Q.id]===void 0&&(m.state.transmissionRenderTarget[Q.id]=new Dn(1,1,{generateMipmaps:!0,type:Te.has("EXT_color_buffer_half_float")||Te.has("EXT_color_buffer_float")?Tr:gn,minFilter:Un,samples:4,stencilBuffer:s,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:je.workingColorSpace}));const ue=m.state.transmissionRenderTarget[Q.id],me=Q.viewport||M;ue.setSize(me.z,me.w);const _e=S.getRenderTarget();S.setRenderTarget(ue),S.getClearColor(O),$=S.getClearAlpha(),$<1&&S.setClearColor(16777215,.5),ve?ce.render(J):S.clear();const Se=S.toneMapping;S.toneMapping=pn;const Pe=Q.viewport;if(Q.viewport!==void 0&&(Q.viewport=void 0),m.setupLightsView(Q),I===!0&&he.setGlobalState(S.clippingPlanes,Q),yn(b,J,Q),Ae.updateMultisampleRenderTarget(ue),Ae.updateRenderTargetMipmap(ue),Te.has("WEBGL_multisampled_render_to_texture")===!1){let Le=!1;for(let Ce=0,qe=F.length;Ce<qe;Ce++){const tt=F[Ce],nt=tt.object,Et=tt.geometry,Ye=tt.material,we=tt.group;if(Ye.side===Nt&&nt.layers.test(Q.layers)){const gt=Ye.side;Ye.side=yt,Ye.needsUpdate=!0,Pa(nt,J,Q,Et,Ye,we),Ye.side=gt,Ye.needsUpdate=!0,Le=!0}}Le===!0&&(Ae.updateMultisampleRenderTarget(ue),Ae.updateRenderTargetMipmap(ue))}S.setRenderTarget(_e),S.setClearColor(O,$),Pe!==void 0&&(Q.viewport=Pe),S.toneMapping=Se}function yn(b,F,J){const Q=F.isScene===!0?F.overrideMaterial:null;for(let W=0,ue=b.length;W<ue;W++){const me=b[W],_e=me.object,Se=me.geometry,Pe=Q===null?me.material:Q,Le=me.group;_e.layers.test(J.layers)&&Pa(_e,F,J,Se,Pe,Le)}}function Pa(b,F,J,Q,W,ue){b.onBeforeRender(S,F,J,Q,W,ue),b.modelViewMatrix.multiplyMatrices(J.matrixWorldInverse,b.matrixWorld),b.normalMatrix.getNormalMatrix(b.modelViewMatrix),W.onBeforeRender(S,F,J,Q,b,ue),W.transparent===!0&&W.side===Nt&&W.forceSinglePass===!1?(W.side=yt,W.needsUpdate=!0,S.renderBufferDirect(J,F,Q,W,b,ue),W.side=mn,W.needsUpdate=!0,S.renderBufferDirect(J,F,Q,W,b,ue),W.side=Nt):S.renderBufferDirect(J,F,Q,W,b,ue),b.onAfterRender(S,F,J,Q,W,ue)}function Ni(b,F,J){F.isScene!==!0&&(F=le);const Q=Ie.get(b),W=m.state.lights,ue=m.state.shadowsArray,me=W.state.version,_e=ae.getParameters(b,W.state,ue,F,J),Se=ae.getProgramCacheKey(_e);let Pe=Q.programs;Q.environment=b.isMeshStandardMaterial?F.environment:null,Q.fog=F.fog,Q.envMap=(b.isMeshStandardMaterial?R:Ve).get(b.envMap||Q.environment),Q.envMapRotation=Q.environment!==null&&b.envMap===null?F.environmentRotation:b.envMapRotation,Pe===void 0&&(b.addEventListener("dispose",se),Pe=new Map,Q.programs=Pe);let Le=Pe.get(Se);if(Le!==void 0){if(Q.currentProgram===Le&&Q.lightsStateVersion===me)return Ua(b,_e),Le}else _e.uniforms=ae.getUniforms(b),b.onBuild(J,_e,S),b.onBeforeCompile(_e,S),Le=ae.acquireProgram(_e,Se),Pe.set(Se,Le),Q.uniforms=_e.uniforms;const Ce=Q.uniforms;return(!b.isShaderMaterial&&!b.isRawShaderMaterial||b.clipping===!0)&&(Ce.clippingPlanes=he.uniform),Ua(b,_e),Q.needsLights=yu(b),Q.lightsStateVersion=me,Q.needsLights&&(Ce.ambientLightColor.value=W.state.ambient,Ce.lightProbe.value=W.state.probe,Ce.directionalLights.value=W.state.directional,Ce.directionalLightShadows.value=W.state.directionalShadow,Ce.spotLights.value=W.state.spot,Ce.spotLightShadows.value=W.state.spotShadow,Ce.rectAreaLights.value=W.state.rectArea,Ce.ltc_1.value=W.state.rectAreaLTC1,Ce.ltc_2.value=W.state.rectAreaLTC2,Ce.pointLights.value=W.state.point,Ce.pointLightShadows.value=W.state.pointShadow,Ce.hemisphereLights.value=W.state.hemi,Ce.directionalShadowMap.value=W.state.directionalShadowMap,Ce.directionalShadowMatrix.value=W.state.directionalShadowMatrix,Ce.spotShadowMap.value=W.state.spotShadowMap,Ce.spotLightMatrix.value=W.state.spotLightMatrix,Ce.spotLightMap.value=W.state.spotLightMap,Ce.pointShadowMap.value=W.state.pointShadowMap,Ce.pointShadowMatrix.value=W.state.pointShadowMatrix),Q.currentProgram=Le,Q.uniformsList=null,Le}function La(b){if(b.uniformsList===null){const F=b.currentProgram.getUniforms();b.uniformsList=fr.seqWithValue(F.seq,b.uniforms)}return b.uniformsList}function Ua(b,F){const J=Ie.get(b);J.outputColorSpace=F.outputColorSpace,J.batching=F.batching,J.batchingColor=F.batchingColor,J.instancing=F.instancing,J.instancingColor=F.instancingColor,J.instancingMorph=F.instancingMorph,J.skinning=F.skinning,J.morphTargets=F.morphTargets,J.morphNormals=F.morphNormals,J.morphColors=F.morphColors,J.morphTargetsCount=F.morphTargetsCount,J.numClippingPlanes=F.numClippingPlanes,J.numIntersection=F.numClipIntersection,J.vertexAlphas=F.vertexAlphas,J.vertexTangents=F.vertexTangents,J.toneMapping=F.toneMapping}function vu(b,F,J,Q,W){F.isScene!==!0&&(F=le),Ae.resetTextureUnits();const ue=F.fog,me=Q.isMeshStandardMaterial?F.environment:null,_e=P===null?S.outputColorSpace:P.isXRRenderTarget===!0?P.texture.colorSpace:vn,Se=(Q.isMeshStandardMaterial?R:Ve).get(Q.envMap||me),Pe=Q.vertexColors===!0&&!!J.attributes.color&&J.attributes.color.itemSize===4,Le=!!J.attributes.tangent&&(!!Q.normalMap||Q.anisotropy>0),Ce=!!J.morphAttributes.position,qe=!!J.morphAttributes.normal,tt=!!J.morphAttributes.color;let nt=pn;Q.toneMapped&&(P===null||P.isXRRenderTarget===!0)&&(nt=S.toneMapping);const Et=J.morphAttributes.position||J.morphAttributes.normal||J.morphAttributes.color,Ye=Et!==void 0?Et.length:0,we=Ie.get(Q),gt=m.state.lights;if(I===!0&&(k===!0||b!==E)){const At=b===E&&Q.id===X;he.setState(Q,b,At)}let $e=!1;Q.version===we.__version?(we.needsLights&&we.lightsStateVersion!==gt.state.version||we.outputColorSpace!==_e||W.isBatchedMesh&&we.batching===!1||!W.isBatchedMesh&&we.batching===!0||W.isBatchedMesh&&we.batchingColor===!0&&W.colorTexture===null||W.isBatchedMesh&&we.batchingColor===!1&&W.colorTexture!==null||W.isInstancedMesh&&we.instancing===!1||!W.isInstancedMesh&&we.instancing===!0||W.isSkinnedMesh&&we.skinning===!1||!W.isSkinnedMesh&&we.skinning===!0||W.isInstancedMesh&&we.instancingColor===!0&&W.instanceColor===null||W.isInstancedMesh&&we.instancingColor===!1&&W.instanceColor!==null||W.isInstancedMesh&&we.instancingMorph===!0&&W.morphTexture===null||W.isInstancedMesh&&we.instancingMorph===!1&&W.morphTexture!==null||we.envMap!==Se||Q.fog===!0&&we.fog!==ue||we.numClippingPlanes!==void 0&&(we.numClippingPlanes!==he.numPlanes||we.numIntersection!==he.numIntersection)||we.vertexAlphas!==Pe||we.vertexTangents!==Le||we.morphTargets!==Ce||we.morphNormals!==qe||we.morphColors!==tt||we.toneMapping!==nt||we.morphTargetsCount!==Ye)&&($e=!0):($e=!0,we.__version=Q.version);let Xt=we.currentProgram;$e===!0&&(Xt=Ni(Q,F,W));let Oi=!1,Sn=!1,Pr=!1;const ct=Xt.getUniforms(),en=we.uniforms;if(Me.useProgram(Xt.program)&&(Oi=!0,Sn=!0,Pr=!0),Q.id!==X&&(X=Q.id,Sn=!0),Oi||E!==b){ct.setValue(L,"projectionMatrix",b.projectionMatrix),ct.setValue(L,"viewMatrix",b.matrixWorldInverse);const At=ct.map.cameraPosition;At!==void 0&&At.setValue(L,q.setFromMatrixPosition(b.matrixWorld)),Xe.logarithmicDepthBuffer&&ct.setValue(L,"logDepthBufFC",2/(Math.log(b.far+1)/Math.LN2)),(Q.isMeshPhongMaterial||Q.isMeshToonMaterial||Q.isMeshLambertMaterial||Q.isMeshBasicMaterial||Q.isMeshStandardMaterial||Q.isShaderMaterial)&&ct.setValue(L,"isOrthographic",b.isOrthographicCamera===!0),E!==b&&(E=b,Sn=!0,Pr=!0)}if(W.isSkinnedMesh){ct.setOptional(L,W,"bindMatrix"),ct.setOptional(L,W,"bindMatrixInverse");const At=W.skeleton;At&&(At.boneTexture===null&&At.computeBoneTexture(),ct.setValue(L,"boneTexture",At.boneTexture,Ae))}W.isBatchedMesh&&(ct.setOptional(L,W,"batchingTexture"),ct.setValue(L,"batchingTexture",W._matricesTexture,Ae),ct.setOptional(L,W,"batchingColorTexture"),W._colorsTexture!==null&&ct.setValue(L,"batchingColorTexture",W._colorsTexture,Ae));const Lr=J.morphAttributes;if((Lr.position!==void 0||Lr.normal!==void 0||Lr.color!==void 0)&&ye.update(W,J,Xt),(Sn||we.receiveShadow!==W.receiveShadow)&&(we.receiveShadow=W.receiveShadow,ct.setValue(L,"receiveShadow",W.receiveShadow)),Q.isMeshGouraudMaterial&&Q.envMap!==null&&(en.envMap.value=Se,en.flipEnvMap.value=Se.isCubeTexture&&Se.isRenderTargetTexture===!1?-1:1),Q.isMeshStandardMaterial&&Q.envMap===null&&F.environment!==null&&(en.envMapIntensity.value=F.environmentIntensity),Sn&&(ct.setValue(L,"toneMappingExposure",S.toneMappingExposure),we.needsLights&&xu(en,Pr),ue&&Q.fog===!0&&oe.refreshFogUniforms(en,ue),oe.refreshMaterialUniforms(en,Q,te,Y,m.state.transmissionRenderTarget[b.id]),fr.upload(L,La(we),en,Ae)),Q.isShaderMaterial&&Q.uniformsNeedUpdate===!0&&(fr.upload(L,La(we),en,Ae),Q.uniformsNeedUpdate=!1),Q.isSpriteMaterial&&ct.setValue(L,"center",W.center),ct.setValue(L,"modelViewMatrix",W.modelViewMatrix),ct.setValue(L,"normalMatrix",W.normalMatrix),ct.setValue(L,"modelMatrix",W.matrixWorld),Q.isShaderMaterial||Q.isRawShaderMaterial){const At=Q.uniformsGroups;for(let Ur=0,Su=At.length;Ur<Su;Ur++){const Da=At[Ur];Fe.update(Da,Xt),Fe.bind(Da,Xt)}}return Xt}function xu(b,F){b.ambientLightColor.needsUpdate=F,b.lightProbe.needsUpdate=F,b.directionalLights.needsUpdate=F,b.directionalLightShadows.needsUpdate=F,b.pointLights.needsUpdate=F,b.pointLightShadows.needsUpdate=F,b.spotLights.needsUpdate=F,b.spotLightShadows.needsUpdate=F,b.rectAreaLights.needsUpdate=F,b.hemisphereLights.needsUpdate=F}function yu(b){return b.isMeshLambertMaterial||b.isMeshToonMaterial||b.isMeshPhongMaterial||b.isMeshStandardMaterial||b.isShadowMaterial||b.isShaderMaterial&&b.lights===!0}this.getActiveCubeFace=function(){return B},this.getActiveMipmapLevel=function(){return D},this.getRenderTarget=function(){return P},this.setRenderTargetTextures=function(b,F,J){Ie.get(b.texture).__webglTexture=F,Ie.get(b.depthTexture).__webglTexture=J;const Q=Ie.get(b);Q.__hasExternalTextures=!0,Q.__autoAllocateDepthBuffer=J===void 0,Q.__autoAllocateDepthBuffer||Te.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),Q.__useRenderToTexture=!1)},this.setRenderTargetFramebuffer=function(b,F){const J=Ie.get(b);J.__webglFramebuffer=F,J.__useDefaultFramebuffer=F===void 0},this.setRenderTarget=function(b,F=0,J=0){P=b,B=F,D=J;let Q=!0,W=null,ue=!1,me=!1;if(b){const Se=Ie.get(b);Se.__useDefaultFramebuffer!==void 0?(Me.bindFramebuffer(L.FRAMEBUFFER,null),Q=!1):Se.__webglFramebuffer===void 0?Ae.setupRenderTarget(b):Se.__hasExternalTextures&&Ae.rebindTextures(b,Ie.get(b.texture).__webglTexture,Ie.get(b.depthTexture).__webglTexture);const Pe=b.texture;(Pe.isData3DTexture||Pe.isDataArrayTexture||Pe.isCompressedArrayTexture)&&(me=!0);const Le=Ie.get(b).__webglFramebuffer;b.isWebGLCubeRenderTarget?(Array.isArray(Le[F])?W=Le[F][J]:W=Le[F],ue=!0):b.samples>0&&Ae.useMultisampledRTT(b)===!1?W=Ie.get(b).__webglMultisampledFramebuffer:Array.isArray(Le)?W=Le[J]:W=Le,M.copy(b.viewport),C.copy(b.scissor),H=b.scissorTest}else M.copy(U).multiplyScalar(te).floor(),C.copy(N).multiplyScalar(te).floor(),H=G;if(Me.bindFramebuffer(L.FRAMEBUFFER,W)&&Q&&Me.drawBuffers(b,W),Me.viewport(M),Me.scissor(C),Me.setScissorTest(H),ue){const Se=Ie.get(b.texture);L.framebufferTexture2D(L.FRAMEBUFFER,L.COLOR_ATTACHMENT0,L.TEXTURE_CUBE_MAP_POSITIVE_X+F,Se.__webglTexture,J)}else if(me){const Se=Ie.get(b.texture),Pe=F||0;L.framebufferTextureLayer(L.FRAMEBUFFER,L.COLOR_ATTACHMENT0,Se.__webglTexture,J||0,Pe)}X=-1},this.readRenderTargetPixels=function(b,F,J,Q,W,ue,me){if(!(b&&b.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let _e=Ie.get(b).__webglFramebuffer;if(b.isWebGLCubeRenderTarget&&me!==void 0&&(_e=_e[me]),_e){Me.bindFramebuffer(L.FRAMEBUFFER,_e);try{const Se=b.texture,Pe=Se.format,Le=Se.type;if(!Xe.textureFormatReadable(Pe)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!Xe.textureTypeReadable(Le)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}F>=0&&F<=b.width-Q&&J>=0&&J<=b.height-W&&L.readPixels(F,J,Q,W,pe.convert(Pe),pe.convert(Le),ue)}finally{const Se=P!==null?Ie.get(P).__webglFramebuffer:null;Me.bindFramebuffer(L.FRAMEBUFFER,Se)}}},this.readRenderTargetPixelsAsync=async function(b,F,J,Q,W,ue,me){if(!(b&&b.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let _e=Ie.get(b).__webglFramebuffer;if(b.isWebGLCubeRenderTarget&&me!==void 0&&(_e=_e[me]),_e){Me.bindFramebuffer(L.FRAMEBUFFER,_e);try{const Se=b.texture,Pe=Se.format,Le=Se.type;if(!Xe.textureFormatReadable(Pe))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!Xe.textureTypeReadable(Le))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");if(F>=0&&F<=b.width-Q&&J>=0&&J<=b.height-W){const Ce=L.createBuffer();L.bindBuffer(L.PIXEL_PACK_BUFFER,Ce),L.bufferData(L.PIXEL_PACK_BUFFER,ue.byteLength,L.STREAM_READ),L.readPixels(F,J,Q,W,pe.convert(Pe),pe.convert(Le),0),L.flush();const qe=L.fenceSync(L.SYNC_GPU_COMMANDS_COMPLETE,0);await nf(L,qe,4);try{L.bindBuffer(L.PIXEL_PACK_BUFFER,Ce),L.getBufferSubData(L.PIXEL_PACK_BUFFER,0,ue)}finally{L.deleteBuffer(Ce),L.deleteSync(qe)}return ue}}finally{const Se=P!==null?Ie.get(P).__webglFramebuffer:null;Me.bindFramebuffer(L.FRAMEBUFFER,Se)}}},this.copyFramebufferToTexture=function(b,F=null,J=0){b.isTexture!==!0&&(console.warn("WebGLRenderer: copyFramebufferToTexture function signature has changed."),F=arguments[0]||null,b=arguments[1]);const Q=Math.pow(2,-J),W=Math.floor(b.image.width*Q),ue=Math.floor(b.image.height*Q),me=F!==null?F.x:0,_e=F!==null?F.y:0;Ae.setTexture2D(b,0),L.copyTexSubImage2D(L.TEXTURE_2D,J,0,0,me,_e,W,ue),Me.unbindTexture()},this.copyTextureToTexture=function(b,F,J=null,Q=null,W=0){b.isTexture!==!0&&(console.warn("WebGLRenderer: copyTextureToTexture function signature has changed."),Q=arguments[0]||null,b=arguments[1],F=arguments[2],W=arguments[3]||0,J=null);let ue,me,_e,Se,Pe,Le;J!==null?(ue=J.max.x-J.min.x,me=J.max.y-J.min.y,_e=J.min.x,Se=J.min.y):(ue=b.image.width,me=b.image.height,_e=0,Se=0),Q!==null?(Pe=Q.x,Le=Q.y):(Pe=0,Le=0);const Ce=pe.convert(F.format),qe=pe.convert(F.type);Ae.setTexture2D(F,0),L.pixelStorei(L.UNPACK_FLIP_Y_WEBGL,F.flipY),L.pixelStorei(L.UNPACK_PREMULTIPLY_ALPHA_WEBGL,F.premultiplyAlpha),L.pixelStorei(L.UNPACK_ALIGNMENT,F.unpackAlignment);const tt=L.getParameter(L.UNPACK_ROW_LENGTH),nt=L.getParameter(L.UNPACK_IMAGE_HEIGHT),Et=L.getParameter(L.UNPACK_SKIP_PIXELS),Ye=L.getParameter(L.UNPACK_SKIP_ROWS),we=L.getParameter(L.UNPACK_SKIP_IMAGES),gt=b.isCompressedTexture?b.mipmaps[W]:b.image;L.pixelStorei(L.UNPACK_ROW_LENGTH,gt.width),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,gt.height),L.pixelStorei(L.UNPACK_SKIP_PIXELS,_e),L.pixelStorei(L.UNPACK_SKIP_ROWS,Se),b.isDataTexture?L.texSubImage2D(L.TEXTURE_2D,W,Pe,Le,ue,me,Ce,qe,gt.data):b.isCompressedTexture?L.compressedTexSubImage2D(L.TEXTURE_2D,W,Pe,Le,gt.width,gt.height,Ce,gt.data):L.texSubImage2D(L.TEXTURE_2D,W,Pe,Le,Ce,qe,gt),L.pixelStorei(L.UNPACK_ROW_LENGTH,tt),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,nt),L.pixelStorei(L.UNPACK_SKIP_PIXELS,Et),L.pixelStorei(L.UNPACK_SKIP_ROWS,Ye),L.pixelStorei(L.UNPACK_SKIP_IMAGES,we),W===0&&F.generateMipmaps&&L.generateMipmap(L.TEXTURE_2D),Me.unbindTexture()},this.copyTextureToTexture3D=function(b,F,J=null,Q=null,W=0){b.isTexture!==!0&&(console.warn("WebGLRenderer: copyTextureToTexture3D function signature has changed."),J=arguments[0]||null,Q=arguments[1]||null,b=arguments[2],F=arguments[3],W=arguments[4]||0);let ue,me,_e,Se,Pe,Le,Ce,qe,tt;const nt=b.isCompressedTexture?b.mipmaps[W]:b.image;J!==null?(ue=J.max.x-J.min.x,me=J.max.y-J.min.y,_e=J.max.z-J.min.z,Se=J.min.x,Pe=J.min.y,Le=J.min.z):(ue=nt.width,me=nt.height,_e=nt.depth,Se=0,Pe=0,Le=0),Q!==null?(Ce=Q.x,qe=Q.y,tt=Q.z):(Ce=0,qe=0,tt=0);const Et=pe.convert(F.format),Ye=pe.convert(F.type);let we;if(F.isData3DTexture)Ae.setTexture3D(F,0),we=L.TEXTURE_3D;else if(F.isDataArrayTexture||F.isCompressedArrayTexture)Ae.setTexture2DArray(F,0),we=L.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}L.pixelStorei(L.UNPACK_FLIP_Y_WEBGL,F.flipY),L.pixelStorei(L.UNPACK_PREMULTIPLY_ALPHA_WEBGL,F.premultiplyAlpha),L.pixelStorei(L.UNPACK_ALIGNMENT,F.unpackAlignment);const gt=L.getParameter(L.UNPACK_ROW_LENGTH),$e=L.getParameter(L.UNPACK_IMAGE_HEIGHT),Xt=L.getParameter(L.UNPACK_SKIP_PIXELS),Oi=L.getParameter(L.UNPACK_SKIP_ROWS),Sn=L.getParameter(L.UNPACK_SKIP_IMAGES);L.pixelStorei(L.UNPACK_ROW_LENGTH,nt.width),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,nt.height),L.pixelStorei(L.UNPACK_SKIP_PIXELS,Se),L.pixelStorei(L.UNPACK_SKIP_ROWS,Pe),L.pixelStorei(L.UNPACK_SKIP_IMAGES,Le),b.isDataTexture||b.isData3DTexture?L.texSubImage3D(we,W,Ce,qe,tt,ue,me,_e,Et,Ye,nt.data):F.isCompressedArrayTexture?L.compressedTexSubImage3D(we,W,Ce,qe,tt,ue,me,_e,Et,nt.data):L.texSubImage3D(we,W,Ce,qe,tt,ue,me,_e,Et,Ye,nt),L.pixelStorei(L.UNPACK_ROW_LENGTH,gt),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,$e),L.pixelStorei(L.UNPACK_SKIP_PIXELS,Xt),L.pixelStorei(L.UNPACK_SKIP_ROWS,Oi),L.pixelStorei(L.UNPACK_SKIP_IMAGES,Sn),W===0&&F.generateMipmaps&&L.generateMipmap(we),Me.unbindTexture()},this.initRenderTarget=function(b){Ie.get(b).__webglFramebuffer===void 0&&Ae.setupRenderTarget(b)},this.initTexture=function(b){b.isCubeTexture?Ae.setTextureCube(b,0):b.isData3DTexture?Ae.setTexture3D(b,0):b.isDataArrayTexture||b.isCompressedArrayTexture?Ae.setTexture2DArray(b,0):Ae.setTexture2D(b,0),Me.unbindTexture()},this.resetState=function(){B=0,D=0,P=null,Me.reset(),Oe.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return Jt}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(e){this._outputColorSpace=e;const t=this.getContext();t.drawingBufferColorSpace=e===wa?"display-p3":"srgb",t.unpackColorSpace=je.workingColorSpace===wr?"display-p3":"srgb"}}class hv extends dt{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new Wt,this.environmentIntensity=1,this.environmentRotation=new Wt,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(e,t){return super.copy(e,t),e.background!==null&&(this.background=e.background.clone()),e.environment!==null&&(this.environment=e.environment.clone()),e.fog!==null&&(this.fog=e.fog.clone()),this.backgroundBlurriness=e.backgroundBlurriness,this.backgroundIntensity=e.backgroundIntensity,this.backgroundRotation.copy(e.backgroundRotation),this.environmentIntensity=e.environmentIntensity,this.environmentRotation.copy(e.environmentRotation),e.overrideMaterial!==null&&(this.overrideMaterial=e.overrideMaterial.clone()),this.matrixAutoUpdate=e.matrixAutoUpdate,this}toJSON(e){const t=super.toJSON(e);return this.fog!==null&&(t.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(t.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(t.object.backgroundIntensity=this.backgroundIntensity),t.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(t.object.environmentIntensity=this.environmentIntensity),t.object.environmentRotation=this.environmentRotation.toArray(),t}}class Ra extends kn{constructor(e){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new He(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.linewidth=e.linewidth,this.linecap=e.linecap,this.linejoin=e.linejoin,this.fog=e.fog,this}}const Sr=new z,Mr=new z,Ql=new Je,Ti=new Ar,sr=new Ii,ia=new z,ec=new z;class dv extends dt{constructor(e=new Lt,t=new Ra){super(),this.isLine=!0,this.type="Line",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,n=[0];for(let r=1,s=t.count;r<s;r++)Sr.fromBufferAttribute(t,r-1),Mr.fromBufferAttribute(t,r),n[r]=n[r-1],n[r]+=Sr.distanceTo(Mr);e.setAttribute("lineDistance",new ft(n,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(e,t){const n=this.geometry,r=this.matrixWorld,s=e.params.Line.threshold,o=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),sr.copy(n.boundingSphere),sr.applyMatrix4(r),sr.radius+=s,e.ray.intersectsSphere(sr)===!1)return;Ql.copy(r).invert(),Ti.copy(e.ray).applyMatrix4(Ql);const a=s/((this.scale.x+this.scale.y+this.scale.z)/3),l=a*a,c=this.isLineSegments?2:1,u=n.index,d=n.attributes.position;if(u!==null){const f=Math.max(0,o.start),g=Math.min(u.count,o.start+o.count);for(let x=f,m=g-1;x<m;x+=c){const p=u.getX(x),T=u.getX(x+1),S=ar(this,e,Ti,l,p,T);S&&t.push(S)}if(this.isLineLoop){const x=u.getX(g-1),m=u.getX(f),p=ar(this,e,Ti,l,x,m);p&&t.push(p)}}else{const f=Math.max(0,o.start),g=Math.min(d.count,o.start+o.count);for(let x=f,m=g-1;x<m;x+=c){const p=ar(this,e,Ti,l,x,x+1);p&&t.push(p)}if(this.isLineLoop){const x=ar(this,e,Ti,l,g-1,f);x&&t.push(x)}}}updateMorphTargets(){const t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){const r=t[n[0]];if(r!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,o=r.length;s<o;s++){const a=r[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=s}}}}}function ar(i,e,t,n,r,s){const o=i.geometry.attributes.position;if(Sr.fromBufferAttribute(o,r),Mr.fromBufferAttribute(o,s),t.distanceSqToSegment(Sr,Mr,ia,ec)>n)return;ia.applyMatrix4(i.matrixWorld);const l=e.ray.origin.distanceTo(ia);if(!(l<e.near||l>e.far))return{distance:l,point:ec.clone().applyMatrix4(i.matrixWorld),index:r,face:null,faceIndex:null,object:i}}const tc=new z,nc=new z;class fu extends dv{constructor(e,t){super(e,t),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,n=[];for(let r=0,s=t.count;r<s;r+=2)tc.fromBufferAttribute(t,r),nc.fromBufferAttribute(t,r+1),n[r]=r===0?0:n[r-1],n[r+1]=n[r]+tc.distanceTo(nc);e.setAttribute("lineDistance",new ft(n,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class pu extends kn{constructor(e){super(),this.isPointsMaterial=!0,this.type="PointsMaterial",this.color=new He(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.alphaMap=e.alphaMap,this.size=e.size,this.sizeAttenuation=e.sizeAttenuation,this.fog=e.fog,this}}const ic=new Je,xa=new Ar,or=new Ii,lr=new z;class fv extends dt{constructor(e=new Lt,t=new pu){super(),this.isPoints=!0,this.type="Points",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}raycast(e,t){const n=this.geometry,r=this.matrixWorld,s=e.params.Points.threshold,o=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),or.copy(n.boundingSphere),or.applyMatrix4(r),or.radius+=s,e.ray.intersectsSphere(or)===!1)return;ic.copy(r).invert(),xa.copy(e.ray).applyMatrix4(ic);const a=s/((this.scale.x+this.scale.y+this.scale.z)/3),l=a*a,c=n.index,h=n.attributes.position;if(c!==null){const d=Math.max(0,o.start),f=Math.min(c.count,o.start+o.count);for(let g=d,x=f;g<x;g++){const m=c.getX(g);lr.fromBufferAttribute(h,m),rc(lr,m,l,r,e,t,this)}}else{const d=Math.max(0,o.start),f=Math.min(h.count,o.start+o.count);for(let g=d,x=f;g<x;g++)lr.fromBufferAttribute(h,g),rc(lr,g,l,r,e,t,this)}}updateMorphTargets(){const t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){const r=t[n[0]];if(r!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,o=r.length;s<o;s++){const a=r[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=s}}}}}function rc(i,e,t,n,r,s,o){const a=xa.distanceSqToPoint(i);if(a<t){const l=new z;xa.closestPointToPoint(i,l),l.applyMatrix4(n);const c=r.ray.origin.distanceTo(l);if(c<r.near||c>r.far)return;s.push({distance:c,distanceToRay:Math.sqrt(a),point:l,index:e,face:null,object:o})}}class sc extends kn{constructor(e){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new He(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new He(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Xc,this.normalScale=new De(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Wt,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.defines={STANDARD:""},this.color.copy(e.color),this.roughness=e.roughness,this.metalness=e.metalness,this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.emissive.copy(e.emissive),this.emissiveMap=e.emissiveMap,this.emissiveIntensity=e.emissiveIntensity,this.bumpMap=e.bumpMap,this.bumpScale=e.bumpScale,this.normalMap=e.normalMap,this.normalMapType=e.normalMapType,this.normalScale.copy(e.normalScale),this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.roughnessMap=e.roughnessMap,this.metalnessMap=e.metalnessMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.envMapIntensity=e.envMapIntensity,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.flatShading=e.flatShading,this.fog=e.fog,this}}class mu extends dt{constructor(e,t=1){super(),this.isLight=!0,this.type="Light",this.color=new He(e),this.intensity=t}dispose(){}copy(e,t){return super.copy(e,t),this.color.copy(e.color),this.intensity=e.intensity,this}toJSON(e){const t=super.toJSON(e);return t.object.color=this.color.getHex(),t.object.intensity=this.intensity,this.groundColor!==void 0&&(t.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(t.object.distance=this.distance),this.angle!==void 0&&(t.object.angle=this.angle),this.decay!==void 0&&(t.object.decay=this.decay),this.penumbra!==void 0&&(t.object.penumbra=this.penumbra),this.shadow!==void 0&&(t.object.shadow=this.shadow.toJSON()),t}}const ra=new Je,ac=new z,oc=new z;class pv{constructor(e){this.camera=e,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new De(512,512),this.map=null,this.mapPass=null,this.matrix=new Je,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new Aa,this._frameExtents=new De(1,1),this._viewportCount=1,this._viewports=[new ht(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(e){const t=this.camera,n=this.matrix;ac.setFromMatrixPosition(e.matrixWorld),t.position.copy(ac),oc.setFromMatrixPosition(e.target.matrixWorld),t.lookAt(oc),t.updateMatrixWorld(),ra.multiplyMatrices(t.projectionMatrix,t.matrixWorldInverse),this._frustum.setFromProjectionMatrix(ra),n.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),n.multiply(ra)}getViewport(e){return this._viewports[e]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(e){return this.camera=e.camera.clone(),this.bias=e.bias,this.radius=e.radius,this.mapSize.copy(e.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const e={};return this.bias!==0&&(e.bias=this.bias),this.normalBias!==0&&(e.normalBias=this.normalBias),this.radius!==1&&(e.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(e.mapSize=this.mapSize.toArray()),e.camera=this.camera.toJSON(!1).object,delete e.camera.matrix,e}}class mv extends pv{constructor(){super(new au(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class gv extends mu{constructor(e,t){super(e,t),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(dt.DEFAULT_UP),this.updateMatrix(),this.target=new dt,this.shadow=new mv}dispose(){this.shadow.dispose()}copy(e){return super.copy(e),this.target=e.target.clone(),this.shadow=e.shadow.clone(),this}}class _v extends mu{constructor(e,t){super(e,t),this.isAmbientLight=!0,this.type="AmbientLight"}}class lc{constructor(e=1,t=0,n=0){return this.radius=e,this.phi=t,this.theta=n,this}set(e,t,n){return this.radius=e,this.phi=t,this.theta=n,this}copy(e){return this.radius=e.radius,this.phi=e.phi,this.theta=e.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(e){return this.setFromCartesianCoords(e.x,e.y,e.z)}setFromCartesianCoords(e,t,n){return this.radius=Math.sqrt(e*e+t*t+n*n),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(e,n),this.phi=Math.acos(vt(t/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}class vv extends fu{constructor(e=10,t=10,n=4473924,r=8947848){n=new He(n),r=new He(r);const s=t/2,o=e/t,a=e/2,l=[],c=[];for(let d=0,f=0,g=-a;d<=t;d++,g+=o){l.push(-a,0,g,a,0,g),l.push(g,0,-a,g,0,a);const x=d===s?n:r;x.toArray(c,f),f+=3,x.toArray(c,f),f+=3,x.toArray(c,f),f+=3,x.toArray(c,f),f+=3}const u=new Lt;u.setAttribute("position",new ft(l,3)),u.setAttribute("color",new ft(c,3));const h=new Ra({vertexColors:!0,toneMapped:!1});super(u,h),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}class sa extends fu{constructor(e=1){const t=[0,0,0,e,0,0,0,0,0,0,e,0,0,0,0,0,0,e],n=[1,0,0,1,.6,0,0,1,0,.6,1,0,0,0,1,0,.6,1],r=new Lt;r.setAttribute("position",new ft(t,3)),r.setAttribute("color",new ft(n,3));const s=new Ra({vertexColors:!0,toneMapped:!1});super(r,s),this.type="AxesHelper"}setColors(e,t,n){const r=new He,s=this.geometry.attributes.color.array;return r.set(e),r.toArray(s,0),r.toArray(s,3),r.set(t),r.toArray(s,6),r.toArray(s,9),r.set(n),r.toArray(s,12),r.toArray(s,15),this.geometry.attributes.color.needsUpdate=!0,this}dispose(){this.geometry.dispose(),this.material.dispose()}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:Ta}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=Ta);const cc={type:"change"},aa={type:"start"},uc={type:"end"},cr=new Ar,hc=new ln,xv=Math.cos(70*ef.DEG2RAD);class yv extends Bn{constructor(e,t){super(),this.object=e,this.domElement=t,this.domElement.style.touchAction="none",this.enabled=!0,this.target=new z,this.cursor=new z,this.minDistance=0,this.maxDistance=1/0,this.minZoom=0,this.maxZoom=1/0,this.minTargetRadius=0,this.maxTargetRadius=1/0,this.minPolarAngle=0,this.maxPolarAngle=Math.PI,this.minAzimuthAngle=-1/0,this.maxAzimuthAngle=1/0,this.enableDamping=!1,this.dampingFactor=.05,this.enableZoom=!0,this.zoomSpeed=1,this.enableRotate=!0,this.rotateSpeed=1,this.enablePan=!0,this.panSpeed=1,this.screenSpacePanning=!0,this.keyPanSpeed=7,this.zoomToCursor=!1,this.autoRotate=!1,this.autoRotateSpeed=2,this.keys={LEFT:"ArrowLeft",UP:"ArrowUp",RIGHT:"ArrowRight",BOTTOM:"ArrowDown"},this.mouseButtons={LEFT:zn.ROTATE,MIDDLE:zn.DOLLY,RIGHT:zn.PAN},this.touches={ONE:Vn.ROTATE,TWO:Vn.DOLLY_PAN},this.target0=this.target.clone(),this.position0=this.object.position.clone(),this.zoom0=this.object.zoom,this._domElementKeyEvents=null,this.getPolarAngle=function(){return a.phi},this.getAzimuthalAngle=function(){return a.theta},this.getDistance=function(){return this.object.position.distanceTo(this.target)},this.listenToKeyEvents=function(_){_.addEventListener("keydown",he),this._domElementKeyEvents=_},this.stopListenToKeyEvents=function(){this._domElementKeyEvents.removeEventListener("keydown",he),this._domElementKeyEvents=null},this.saveState=function(){n.target0.copy(n.target),n.position0.copy(n.object.position),n.zoom0=n.object.zoom},this.reset=function(){n.target.copy(n.target0),n.object.position.copy(n.position0),n.object.zoom=n.zoom0,n.object.updateProjectionMatrix(),n.dispatchEvent(cc),n.update(),s=r.NONE},this.update=function(){const _=new z,ne=new Ht().setFromUnitVectors(e.up,new z(0,1,0)),Z=ne.clone().invert(),ie=new z,se=new Ht,Ee=new z,Ue=2*Math.PI;return function(it=null){const We=n.object.position;_.copy(We).sub(n.target),_.applyQuaternion(ne),a.setFromVector3(_),n.autoRotate&&s===r.NONE&&H(M(it)),n.enableDamping?(a.theta+=l.theta*n.dampingFactor,a.phi+=l.phi*n.dampingFactor):(a.theta+=l.theta,a.phi+=l.phi);let rt=n.minAzimuthAngle,st=n.maxAzimuthAngle;isFinite(rt)&&isFinite(st)&&(rt<-Math.PI?rt+=Ue:rt>Math.PI&&(rt-=Ue),st<-Math.PI?st+=Ue:st>Math.PI&&(st-=Ue),rt<=st?a.theta=Math.max(rt,Math.min(st,a.theta)):a.theta=a.theta>(rt+st)/2?Math.max(rt,a.theta):Math.min(st,a.theta)),a.phi=Math.max(n.minPolarAngle,Math.min(n.maxPolarAngle,a.phi)),a.makeSafe(),n.enableDamping===!0?n.target.addScaledVector(u,n.dampingFactor):n.target.add(u),n.target.sub(n.cursor),n.target.clampLength(n.minTargetRadius,n.maxTargetRadius),n.target.add(n.cursor);let Mt=!1;if(n.zoomToCursor&&D||n.object.isOrthographicCamera)a.radius=U(a.radius);else{const bt=a.radius;a.radius=U(a.radius*c),Mt=bt!=a.radius}if(_.setFromSpherical(a),_.applyQuaternion(Z),We.copy(n.target).add(_),n.object.lookAt(n.target),n.enableDamping===!0?(l.theta*=1-n.dampingFactor,l.phi*=1-n.dampingFactor,u.multiplyScalar(1-n.dampingFactor)):(l.set(0,0,0),u.set(0,0,0)),n.zoomToCursor&&D){let bt=null;if(n.object.isPerspectiveCamera){const Qt=_.length();bt=U(Qt*c);const xn=Qt-bt;n.object.position.addScaledVector(A,xn),n.object.updateMatrixWorld(),Mt=!!xn}else if(n.object.isOrthographicCamera){const Qt=new z(B.x,B.y,0);Qt.unproject(n.object);const xn=n.object.zoom;n.object.zoom=Math.max(n.minZoom,Math.min(n.maxZoom,n.object.zoom/c)),n.object.updateProjectionMatrix(),Mt=xn!==n.object.zoom;const yn=new z(B.x,B.y,0);yn.unproject(n.object),n.object.position.sub(yn).add(Qt),n.object.updateMatrixWorld(),bt=_.length()}else console.warn("WARNING: OrbitControls.js encountered an unknown camera type - zoom to cursor disabled."),n.zoomToCursor=!1;bt!==null&&(this.screenSpacePanning?n.target.set(0,0,-1).transformDirection(n.object.matrix).multiplyScalar(bt).add(n.object.position):(cr.origin.copy(n.object.position),cr.direction.set(0,0,-1).transformDirection(n.object.matrix),Math.abs(n.object.up.dot(cr.direction))<xv?e.lookAt(n.target):(hc.setFromNormalAndCoplanarPoint(n.object.up,n.target),cr.intersectPlane(hc,n.target))))}else if(n.object.isOrthographicCamera){const bt=n.object.zoom;n.object.zoom=Math.max(n.minZoom,Math.min(n.maxZoom,n.object.zoom/c)),bt!==n.object.zoom&&(n.object.updateProjectionMatrix(),Mt=!0)}return c=1,D=!1,Mt||ie.distanceToSquared(n.object.position)>o||8*(1-se.dot(n.object.quaternion))>o||Ee.distanceToSquared(n.target)>o?(n.dispatchEvent(cc),ie.copy(n.object.position),se.copy(n.object.quaternion),Ee.copy(n.target),!0):!1}}(),this.dispose=function(){n.domElement.removeEventListener("contextmenu",ye),n.domElement.removeEventListener("pointerdown",Ve),n.domElement.removeEventListener("pointercancel",y),n.domElement.removeEventListener("wheel",ae),n.domElement.removeEventListener("pointermove",R),n.domElement.removeEventListener("pointerup",y),n.domElement.getRootNode().removeEventListener("keydown",be,{capture:!0}),n._domElementKeyEvents!==null&&(n._domElementKeyEvents.removeEventListener("keydown",he),n._domElementKeyEvents=null)};const n=this,r={NONE:-1,ROTATE:0,DOLLY:1,PAN:2,TOUCH_ROTATE:3,TOUCH_PAN:4,TOUCH_DOLLY_PAN:5,TOUCH_DOLLY_ROTATE:6};let s=r.NONE;const o=1e-6,a=new lc,l=new lc;let c=1;const u=new z,h=new De,d=new De,f=new De,g=new De,x=new De,m=new De,p=new De,T=new De,S=new De,A=new z,B=new De;let D=!1;const P=[],X={};let E=!1;function M(_){return _!==null?2*Math.PI/60*n.autoRotateSpeed*_:2*Math.PI/60/60*n.autoRotateSpeed}function C(_){const ne=Math.abs(_*.01);return Math.pow(.95,n.zoomSpeed*ne)}function H(_){l.theta-=_}function O(_){l.phi-=_}const $=function(){const _=new z;return function(Z,ie){_.setFromMatrixColumn(ie,0),_.multiplyScalar(-Z),u.add(_)}}(),V=function(){const _=new z;return function(Z,ie){n.screenSpacePanning===!0?_.setFromMatrixColumn(ie,1):(_.setFromMatrixColumn(ie,0),_.crossVectors(n.object.up,_)),_.multiplyScalar(Z),u.add(_)}}(),Y=function(){const _=new z;return function(Z,ie){const se=n.domElement;if(n.object.isPerspectiveCamera){const Ee=n.object.position;_.copy(Ee).sub(n.target);let Ue=_.length();Ue*=Math.tan(n.object.fov/2*Math.PI/180),$(2*Z*Ue/se.clientHeight,n.object.matrix),V(2*ie*Ue/se.clientHeight,n.object.matrix)}else n.object.isOrthographicCamera?($(Z*(n.object.right-n.object.left)/n.object.zoom/se.clientWidth,n.object.matrix),V(ie*(n.object.top-n.object.bottom)/n.object.zoom/se.clientHeight,n.object.matrix)):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - pan disabled."),n.enablePan=!1)}}();function te(_){n.object.isPerspectiveCamera||n.object.isOrthographicCamera?c/=_:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),n.enableZoom=!1)}function v(_){n.object.isPerspectiveCamera||n.object.isOrthographicCamera?c*=_:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),n.enableZoom=!1)}function w(_,ne){if(!n.zoomToCursor)return;D=!0;const Z=n.domElement.getBoundingClientRect(),ie=_-Z.left,se=ne-Z.top,Ee=Z.width,Ue=Z.height;B.x=ie/Ee*2-1,B.y=-(se/Ue)*2+1,A.set(B.x,B.y,1).unproject(n.object).sub(n.object.position).normalize()}function U(_){return Math.max(n.minDistance,Math.min(n.maxDistance,_))}function N(_){h.set(_.clientX,_.clientY)}function G(_){w(_.clientX,_.clientX),p.set(_.clientX,_.clientY)}function K(_){g.set(_.clientX,_.clientY)}function I(_){d.set(_.clientX,_.clientY),f.subVectors(d,h).multiplyScalar(n.rotateSpeed);const ne=n.domElement;H(2*Math.PI*f.x/ne.clientHeight),O(2*Math.PI*f.y/ne.clientHeight),h.copy(d),n.update()}function k(_){T.set(_.clientX,_.clientY),S.subVectors(T,p),S.y>0?te(C(S.y)):S.y<0&&v(C(S.y)),p.copy(T),n.update()}function j(_){x.set(_.clientX,_.clientY),m.subVectors(x,g).multiplyScalar(n.panSpeed),Y(m.x,m.y),g.copy(x),n.update()}function q(_){w(_.clientX,_.clientY),_.deltaY<0?v(C(_.deltaY)):_.deltaY>0&&te(C(_.deltaY)),n.update()}function le(_){let ne=!1;switch(_.code){case n.keys.UP:_.ctrlKey||_.metaKey||_.shiftKey?O(2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):Y(0,n.keyPanSpeed),ne=!0;break;case n.keys.BOTTOM:_.ctrlKey||_.metaKey||_.shiftKey?O(-2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):Y(0,-n.keyPanSpeed),ne=!0;break;case n.keys.LEFT:_.ctrlKey||_.metaKey||_.shiftKey?H(2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):Y(n.keyPanSpeed,0),ne=!0;break;case n.keys.RIGHT:_.ctrlKey||_.metaKey||_.shiftKey?H(-2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):Y(-n.keyPanSpeed,0),ne=!0;break}ne&&(_.preventDefault(),n.update())}function ve(_){if(P.length===1)h.set(_.pageX,_.pageY);else{const ne=Fe(_),Z=.5*(_.pageX+ne.x),ie=.5*(_.pageY+ne.y);h.set(Z,ie)}}function ge(_){if(P.length===1)g.set(_.pageX,_.pageY);else{const ne=Fe(_),Z=.5*(_.pageX+ne.x),ie=.5*(_.pageY+ne.y);g.set(Z,ie)}}function L(_){const ne=Fe(_),Z=_.pageX-ne.x,ie=_.pageY-ne.y,se=Math.sqrt(Z*Z+ie*ie);p.set(0,se)}function xe(_){n.enableZoom&&L(_),n.enablePan&&ge(_)}function Te(_){n.enableZoom&&L(_),n.enableRotate&&ve(_)}function Xe(_){if(P.length==1)d.set(_.pageX,_.pageY);else{const Z=Fe(_),ie=.5*(_.pageX+Z.x),se=.5*(_.pageY+Z.y);d.set(ie,se)}f.subVectors(d,h).multiplyScalar(n.rotateSpeed);const ne=n.domElement;H(2*Math.PI*f.x/ne.clientHeight),O(2*Math.PI*f.y/ne.clientHeight),h.copy(d)}function Me(_){if(P.length===1)x.set(_.pageX,_.pageY);else{const ne=Fe(_),Z=.5*(_.pageX+ne.x),ie=.5*(_.pageY+ne.y);x.set(Z,ie)}m.subVectors(x,g).multiplyScalar(n.panSpeed),Y(m.x,m.y),g.copy(x)}function Be(_){const ne=Fe(_),Z=_.pageX-ne.x,ie=_.pageY-ne.y,se=Math.sqrt(Z*Z+ie*ie);T.set(0,se),S.set(0,Math.pow(T.y/p.y,n.zoomSpeed)),te(S.y),p.copy(T);const Ee=(_.pageX+ne.x)*.5,Ue=(_.pageY+ne.y)*.5;w(Ee,Ue)}function Ie(_){n.enableZoom&&Be(_),n.enablePan&&Me(_)}function Ae(_){n.enableZoom&&Be(_),n.enableRotate&&Xe(_)}function Ve(_){n.enabled!==!1&&(P.length===0&&(n.domElement.setPointerCapture(_.pointerId),n.domElement.addEventListener("pointermove",R),n.domElement.addEventListener("pointerup",y)),!pe(_)&&(Ge(_),_.pointerType==="touch"?Ne(_):ee(_)))}function R(_){n.enabled!==!1&&(_.pointerType==="touch"?ce(_):re(_))}function y(_){switch(Re(_),P.length){case 0:n.domElement.releasePointerCapture(_.pointerId),n.domElement.removeEventListener("pointermove",R),n.domElement.removeEventListener("pointerup",y),n.dispatchEvent(uc),s=r.NONE;break;case 1:const ne=P[0],Z=X[ne];Ne({pointerId:ne,pageX:Z.x,pageY:Z.y});break}}function ee(_){let ne;switch(_.button){case 0:ne=n.mouseButtons.LEFT;break;case 1:ne=n.mouseButtons.MIDDLE;break;case 2:ne=n.mouseButtons.RIGHT;break;default:ne=-1}switch(ne){case zn.DOLLY:if(n.enableZoom===!1)return;G(_),s=r.DOLLY;break;case zn.ROTATE:if(_.ctrlKey||_.metaKey||_.shiftKey){if(n.enablePan===!1)return;K(_),s=r.PAN}else{if(n.enableRotate===!1)return;N(_),s=r.ROTATE}break;case zn.PAN:if(_.ctrlKey||_.metaKey||_.shiftKey){if(n.enableRotate===!1)return;N(_),s=r.ROTATE}else{if(n.enablePan===!1)return;K(_),s=r.PAN}break;default:s=r.NONE}s!==r.NONE&&n.dispatchEvent(aa)}function re(_){switch(s){case r.ROTATE:if(n.enableRotate===!1)return;I(_);break;case r.DOLLY:if(n.enableZoom===!1)return;k(_);break;case r.PAN:if(n.enablePan===!1)return;j(_);break}}function ae(_){n.enabled===!1||n.enableZoom===!1||s!==r.NONE||(_.preventDefault(),n.dispatchEvent(aa),q(oe(_)),n.dispatchEvent(uc))}function oe(_){const ne=_.deltaMode,Z={clientX:_.clientX,clientY:_.clientY,deltaY:_.deltaY};switch(ne){case 1:Z.deltaY*=16;break;case 2:Z.deltaY*=100;break}return _.ctrlKey&&!E&&(Z.deltaY*=10),Z}function be(_){_.key==="Control"&&(E=!0,n.domElement.getRootNode().addEventListener("keyup",de,{passive:!0,capture:!0}))}function de(_){_.key==="Control"&&(E=!1,n.domElement.getRootNode().removeEventListener("keyup",de,{passive:!0,capture:!0}))}function he(_){n.enabled===!1||n.enablePan===!1||le(_)}function Ne(_){switch(Oe(_),P.length){case 1:switch(n.touches.ONE){case Vn.ROTATE:if(n.enableRotate===!1)return;ve(_),s=r.TOUCH_ROTATE;break;case Vn.PAN:if(n.enablePan===!1)return;ge(_),s=r.TOUCH_PAN;break;default:s=r.NONE}break;case 2:switch(n.touches.TWO){case Vn.DOLLY_PAN:if(n.enableZoom===!1&&n.enablePan===!1)return;xe(_),s=r.TOUCH_DOLLY_PAN;break;case Vn.DOLLY_ROTATE:if(n.enableZoom===!1&&n.enableRotate===!1)return;Te(_),s=r.TOUCH_DOLLY_ROTATE;break;default:s=r.NONE}break;default:s=r.NONE}s!==r.NONE&&n.dispatchEvent(aa)}function ce(_){switch(Oe(_),s){case r.TOUCH_ROTATE:if(n.enableRotate===!1)return;Xe(_),n.update();break;case r.TOUCH_PAN:if(n.enablePan===!1)return;Me(_),n.update();break;case r.TOUCH_DOLLY_PAN:if(n.enableZoom===!1&&n.enablePan===!1)return;Ie(_),n.update();break;case r.TOUCH_DOLLY_ROTATE:if(n.enableZoom===!1&&n.enableRotate===!1)return;Ae(_),n.update();break;default:s=r.NONE}}function ye(_){n.enabled!==!1&&_.preventDefault()}function Ge(_){P.push(_.pointerId)}function Re(_){delete X[_.pointerId];for(let ne=0;ne<P.length;ne++)if(P[ne]==_.pointerId){P.splice(ne,1);return}}function pe(_){for(let ne=0;ne<P.length;ne++)if(P[ne]==_.pointerId)return!0;return!1}function Oe(_){let ne=X[_.pointerId];ne===void 0&&(ne=new De,X[_.pointerId]=ne),ne.set(_.pageX,_.pageY)}function Fe(_){const ne=_.pointerId===P[0]?P[1]:P[0];return X[ne]}n.domElement.addEventListener("contextmenu",ye),n.domElement.addEventListener("pointerdown",Ve),n.domElement.addEventListener("pointercancel",y),n.domElement.addEventListener("wheel",ae,{passive:!1}),n.domElement.getRootNode().addEventListener("keydown",be,{passive:!0,capture:!0}),this.update()}}const Sv="cabin_frame",si="Scepter_depth_frame",oa="gripper_frame",la=.7,ca={x:.2,y:.1,z:.2},Mv=new z(2.6,-3,2),bv=new z(0,0,.15),Ev=new z(-.85,-1.25,.55),Tv=new z(.18,0,.04),wv=new z(3.6,-4.1,2.8),Av=new z(0,0,.1);function ur(i){const e=new Lt;e.setAttribute("position",new ft([],3));const t=new pu({color:i,size:.035,transparent:!0,opacity:.78,sizeAttenuation:!0});return new fv(e,t)}function Cv(i){return new z(Number((i==null?void 0:i.x)||0),Number((i==null?void 0:i.y)||0),Number((i==null?void 0:i.z)||0))}function gu(i,e,t=new Map,n=new Set){if(t.has(i))return t.get(i);if(i===Sv){const l={position:new z(0,0,0),quaternion:new Ht};return t.set(i,l),l}if(n.has(i))return null;const r=e.get(i);if(!r)return null;n.add(i);const s=gu(r.parentFrame,e,t,n)||{position:new z(0,0,0),quaternion:new Ht},o=r.position.clone().applyQuaternion(s.quaternion),a={position:s.position.clone().add(o),quaternion:s.quaternion.clone().multiply(r.quaternion.clone())};return t.set(i,a),n.delete(i),a}function Rv(i){return new z(i.y,-i.x,i.z)}class Pv{constructor({container:e}){this.container=e,this.layerState={},this.transformMap=new Map,this.cachedWorldTransforms=new Map,this.sourcePointCloudPositions={filteredWorldCoord:new Float32Array,rawWorldCoord:new Float32Array},this.pointCounts={filteredWorldCoord:0,rawWorldCoord:0,tiePoints:0,planningPoints:0},this.scene=new hv,this.scene.background=new He(462872),this.scene.up.set(0,0,1),this.camera=new Rt(55,1,.01,200),this.camera.up.set(0,0,1),this.camera.position.copy(Mv),this.renderer=new uv({antialias:!0,alpha:!0}),this.renderer.setPixelRatio(window.devicePixelRatio||1),this.container.appendChild(this.renderer.domElement),this.controls=new yv(this.camera,this.renderer.domElement),this.controls.enableDamping=!0,this.controls.target.copy(bv),this.controls.minDistance=.25,this.controls.maxDistance=24,this.controls.minPolarAngle=.08,this.controls.maxPolarAngle=Math.PI*.48,this.ambientLight=new _v(16777215,1.1),this.keyLight=new gv(16777215,.9),this.keyLight.position.set(3,-2,4),this.scene.add(this.ambientLight,this.keyLight),this.grid=new vv(8,16,2509919,1454141),this.grid.rotation.x=Math.PI/2,this.scene.add(this.grid),this.cabinAxes=new sa(.4),this.scene.add(this.cabinAxes),this.scepterFrame=new dn,this.scepterFrame.add(new sa(.28)),this.scene.add(this.scepterFrame),this.gripperFrame=new dn,this.gripperFrame.add(new sa(.22)),this.scene.add(this.gripperFrame),this.robotGroup=new dn;const t=new sc({color:5015551,metalness:.1,roughness:.52,transparent:!0,opacity:.42,side:Nt}),n=new Ft(new In(la,la,la),t);this.robotGroup.add(n),this.scene.add(this.robotGroup),this.tcpToolGroup=new dn;const r=new sc({color:16747597,metalness:.08,roughness:.4,transparent:!0,opacity:.78,side:Nt}),s=new Ft(new In(ca.x,ca.y,ca.z),r);this.tcpToolGroup.add(s),this.scene.add(this.tcpToolGroup),this.filteredPointCloud=ur(6477567),this.rawPointCloud=ur(5204861),this.tiePoints=ur(5767069),this.planningPoints=ur(16307298),this.scene.add(this.filteredPointCloud,this.rawPointCloud,this.tiePoints,this.planningPoints),this.viewMode="camera",this.followCamera=!1,this.needsViewReset=!0,this.resizeObserver=new ResizeObserver(()=>this.resize()),this.resizeObserver.observe(this.container),this.resize(),this.startRenderLoop()}startRenderLoop(){const e=()=>{this.controls.update(),this.followCamera&&this.resetView(this.viewMode),this.needsViewReset&&(this.resetView(this.viewMode),this.needsViewReset=!1),this.renderer.render(this.scene,this.camera),this.rafId=window.requestAnimationFrame(e)};e()}resize(){const e=Math.max(this.container.clientWidth,280),t=Math.max(this.container.clientHeight,220);this.camera.aspect=e/t,this.camera.updateProjectionMatrix(),this.renderer.setSize(e,t)}setLayerState(e){this.layerState={...e},this.robotGroup.visible=!!e.showRobot,this.tcpToolGroup.visible=!!e.showRobot;const t=!!e.showAxes;this.cabinAxes.visible=t,this.scepterFrame.visible=t,this.gripperFrame.visible=t,this.grid.visible=t,this.filteredPointCloud.visible=!!e.showPointCloud&&e.pointCloudSource==="filteredWorldCoord",this.rawPointCloud.visible=!!e.showPointCloud&&e.pointCloudSource==="rawWorldCoord",this.tiePoints.visible=!!e.showTiePoints,this.planningPoints.visible=!!e.showPlanningMarkers,[this.filteredPointCloud,this.rawPointCloud,this.tiePoints,this.planningPoints].forEach(n=>{n.material.size=Number(e.pointSize)||.035,n.material.opacity=Number(e.pointOpacity)||.78,n.material.needsUpdate=!0})}setViewMode(e){this.viewMode=e,this.needsViewReset=!0}setFollowCamera(e){this.followCamera=!!e,this.needsViewReset=!0}resetView(e=this.viewMode){const t=this.getWorldTransform(si);if(e==="camera"&&t){const n=Ev.clone().applyQuaternion(t.quaternion),r=Tv.clone().applyQuaternion(t.quaternion),s=t.position.clone().add(n),o=t.position.clone().add(r);this.camera.position.lerp(s,.28),this.controls.target.lerp(o,.28);return}this.camera.position.lerp(wv,.28),this.controls.target.lerp(Av,.28)}handleTfMessage(e){(Array.isArray(e==null?void 0:e.transforms)?e.transforms:[]).forEach(n=>{var o,a,l,c,u,h,d,f,g,x,m,p,T,S,A;const r=n==null?void 0:n.child_frame_id,s=(o=n==null?void 0:n.header)==null?void 0:o.frame_id;!r||!s||this.transformMap.set(r,{parentFrame:s,position:new z(Number(((l=(a=n.transform)==null?void 0:a.translation)==null?void 0:l.x)||0),Number(((u=(c=n.transform)==null?void 0:c.translation)==null?void 0:u.y)||0),Number(((d=(h=n.transform)==null?void 0:h.translation)==null?void 0:d.z)||0)),quaternion:new Ht(Number(((g=(f=n.transform)==null?void 0:f.rotation)==null?void 0:g.x)||0),Number(((m=(x=n.transform)==null?void 0:x.rotation)==null?void 0:m.y)||0),Number(((T=(p=n.transform)==null?void 0:p.rotation)==null?void 0:T.z)||0),Number(((A=(S=n.transform)==null?void 0:S.rotation)==null?void 0:A.w)||1))})}),this.cachedWorldTransforms.clear(),this.applyFrameTransforms(),this.refreshPointCloudWorldPositions()}applyFrameTransforms(){this.applyGroupTransform(this.scepterFrame,this.getWorldTransform(si));const e=this.getWorldTransform(oa);this.applyGroupTransform(this.gripperFrame,e);const t=this.getWorldTransform(si);t&&(this.robotGroup.position.copy(t.position),this.robotGroup.quaternion.copy(t.quaternion)),e?(this.tcpToolGroup.visible=this.layerState.showRobot!==!1,this.tcpToolGroup.position.copy(e.position),this.tcpToolGroup.quaternion.copy(e.quaternion)):this.tcpToolGroup.visible=!1}applyGroupTransform(e,t){if(!t){e.visible=!1;return}this.layerState.showAxes!==!1&&(e.visible=!0),e.position.copy(t.position),e.quaternion.copy(t.quaternion)}getWorldTransform(e){return gu(e,this.transformMap,this.cachedWorldTransforms)}getKnownTransformCount(){return this.transformMap.size}refreshPointCloudWorldPositions(){this.applyPointCloudWorldPositions("filteredWorldCoord"),this.applyPointCloudWorldPositions("rawWorldCoord")}getGripperOffsetRelativeToScepter(){const e=this.transformMap.get(oa);if((e==null?void 0:e.parentFrame)===si)return e.position.z;const t=this.getWorldTransform(si),n=this.getWorldTransform(oa);return!t||!n?0:n.position.z-t.position.z}convertScepterPointCloudPointToCabinPoint(e){const t=this.getWorldTransform(si);if(!t)return e.clone();const n=Rv(e),r=this.getGripperOffsetRelativeToScepter();return new z(t.position.x+n.x,t.position.y+n.y,t.position.z-n.z-r)}applyPointCloudWorldPositions(e){const t=this.sourcePointCloudPositions[e];if(!(t!=null&&t.length))return;const n=Float32Array.from(t);for(let s=0;s<n.length;s+=3){const o=new z(n[s],n[s+1],n[s+2]),a=this.convertScepterPointCloudPointToCabinPoint(o);n[s]=a.x,n[s+1]=a.y,n[s+2]=a.z}const r=e==="rawWorldCoord"?this.rawPointCloud:this.filteredPointCloud;r.geometry.setAttribute("position",new ft(n,3)),r.geometry.computeBoundingSphere()}setTiePointsMessage(e){const t=Array.isArray(e==null?void 0:e.PointCoordinatesArray)?e.PointCoordinatesArray:[],n=[];return t.forEach(r=>{const s=Array.isArray(r==null?void 0:r.World_coord)?r.World_coord:[];if(s.length<3)return;const o=Cv({x:Number(s[0])/1e3,y:Number(s[1])/1e3,z:Number(s[2])/1e3});n.push(o.x,o.y,o.z)}),this.tiePoints.geometry.setAttribute("position",new ft(n,3)),this.tiePoints.geometry.computeBoundingSphere(),this.pointCounts.tiePoints=n.length/3,this.pointCounts.tiePoints}setPlanningMarkersMessage(e){const t=Array.isArray(e==null?void 0:e.markers)?e.markers:[],n=[];return t.forEach(r=>{(Array.isArray(r==null?void 0:r.points)?r.points:[]).forEach(o=>{n.push(Number((o==null?void 0:o.x)||0),Number((o==null?void 0:o.y)||0),Number((o==null?void 0:o.z)||0))})}),this.planningPoints.geometry.setAttribute("position",new ft(n,3)),this.planningPoints.geometry.computeBoundingSphere(),this.pointCounts.planningPoints=n.length/3,this.pointCounts.planningPoints}setPointCloudImageMessage(e,t){const{positions:n,count:r}=Oh(t,{sampleStep:e==="rawWorldCoord"?8:5,maxPoints:e==="rawWorldCoord"?12e3:18e3});return this.sourcePointCloudPositions[e]=n,this.applyPointCloudWorldPositions(e),this.pointCounts[e]=r,r}}class Lv{constructor({canvas:e,overlayCanvas:t,onSelectionChanged:n,onMessage:r}){this.canvas=e,this.overlayCanvas=t,this.ctx=e.getContext("2d"),this.overlayCtx=t.getContext("2d"),this.onSelectionChanged=n,this.onMessage=r,this.lastImageMessage=null,this.lastExecutionResultMessage=null,this.savedWorkspacePoints=[],this.selectedPoints=[],this.displaySettings={mode:"auto",gamma:.85,overlayOpacity:.88},this.dragState={activeIndex:-1,moved:!1},this.suppressNextCanvasClick=!1}bindPointerEvents(){this.canvas.addEventListener("click",e=>this.handleCanvasClick(e)),this.canvas.addEventListener("pointerdown",e=>this.handlePointerDown(e)),this.canvas.addEventListener("pointermove",e=>this.handlePointerMove(e)),this.canvas.addEventListener("pointerup",()=>this.handlePointerUp()),this.canvas.addEventListener("pointerleave",()=>this.handlePointerUp())}setDisplaySettings(e){this.displaySettings={...this.displaySettings,...e},this.overlayCanvas.style.opacity=String(this.displaySettings.overlayOpacity),this.draw()}setBaseImageMessage(e){this.lastImageMessage=e,this.draw()}setS2OverlayMessage(e){this.setExecutionOverlayMessage(e)}setExecutionOverlayMessage(e){this.lastExecutionResultMessage=e,this.drawOverlay()}setOverlaySource(e){this.drawOverlay()}setSavedWorkspacePoints(e){this.savedWorkspacePoints=Array.isArray(e)?e:[],this.draw()}setSavedWorkspacePayload(e){this.savedWorkspacePoints=Rh(e),this.draw()}getSelectedPoints(){return[...this.selectedPoints]}getSavedWorkspacePoints(){return[...this.savedWorkspacePoints]}clearSelection(){this.selectedPoints=[],this.notifySelectionChanged(),this.draw()}undoSelection(){this.selectedPoints.length&&(this.selectedPoints=this.selectedPoints.slice(0,-1),this.notifySelectionChanged(),this.draw())}buildWorkspacePayload(){return Rc(this.selectedPoints)}notifySelectionChanged(){var e;(e=this.onSelectionChanged)==null||e.call(this,this.getSelectedPoints())}draw(){if(!this.lastImageMessage)return;const e=Po(this.lastImageMessage,this.displaySettings);this.canvas.width=e.width,this.canvas.height=e.height,this.overlayCanvas.width=e.width,this.overlayCanvas.height=e.height,this.ctx.putImageData(e,0,0),this.drawWorkspacePolylines(),this.drawOverlay()}drawOverlay(){if(this.overlayCtx.clearRect(0,0,this.overlayCanvas.width,this.overlayCanvas.height),!this.lastExecutionResultMessage)return;const e=Po(this.lastExecutionResultMessage,{mode:"raw",gamma:1});this.overlayCanvas.width=e.width,this.overlayCanvas.height=e.height,this.overlayCtx.putImageData(e,0,0),this.overlayCanvas.style.opacity=String(this.displaySettings.overlayOpacity)}drawWorkspacePolylines(){if(this.ctx.save(),this.ctx.lineWidth=2,this.ctx.font="18px monospace",this.savedWorkspacePoints.length>=2){this.ctx.strokeStyle="#6aa6ff",this.ctx.fillStyle="#6aa6ff",this.ctx.setLineDash([10,6]),this.ctx.beginPath(),this.ctx.moveTo(this.savedWorkspacePoints[0].x,this.savedWorkspacePoints[0].y);for(let e=1;e<this.savedWorkspacePoints.length;e+=1)this.ctx.lineTo(this.savedWorkspacePoints[e].x,this.savedWorkspacePoints[e].y);this.savedWorkspacePoints.length===4&&this.ctx.closePath(),this.ctx.stroke(),this.ctx.setLineDash([])}if(this.selectedPoints.length){if(this.ctx.strokeStyle="#4de3a5",this.ctx.fillStyle="#ffae42",this.selectedPoints.length>=2){this.ctx.beginPath(),this.ctx.moveTo(this.selectedPoints[0].x,this.selectedPoints[0].y);for(let e=1;e<this.selectedPoints.length;e+=1)this.ctx.lineTo(this.selectedPoints[e].x,this.selectedPoints[e].y);this.selectedPoints.length===4&&this.ctx.closePath(),this.ctx.stroke()}this.selectedPoints.forEach((e,t)=>{this.ctx.beginPath(),this.ctx.arc(e.x,e.y,5,0,Math.PI*2),this.ctx.fill(),this.ctx.strokeStyle="#ffffff",this.ctx.stroke(),this.ctx.fillStyle="#4de3a5",this.ctx.fillText(`${t+1}`,e.x+8,e.y-8),this.ctx.fillStyle="#ffae42"})}this.ctx.restore()}handleCanvasClick(e){var n,r;if(this.suppressNextCanvasClick){this.suppressNextCanvasClick=!1;return}if(!this.lastImageMessage){(n=this.onMessage)==null||n.call(this,"IR 图像还没到，先等一帧");return}if(this.selectedPoints.length>=4){(r=this.onMessage)==null||r.call(this,"已经点满 4 个角点了，先清空或撤销再继续");return}const t=bs({clientX:e.clientX,clientY:e.clientY,rect:this.canvas.getBoundingClientRect(),imageWidth:Number(this.lastImageMessage.width),imageHeight:Number(this.lastImageMessage.height)});this.selectedPoints=[...this.selectedPoints,t],this.notifySelectionChanged(),this.draw()}handlePointerDown(e){var r,s;if(!this.lastImageMessage||!this.selectedPoints.length)return;const t=bs({clientX:e.clientX,clientY:e.clientY,rect:this.canvas.getBoundingClientRect(),imageWidth:Number(this.lastImageMessage.width),imageHeight:Number(this.lastImageMessage.height)}),n=Ph(this.selectedPoints,t,12);n<0||(this.dragState={activeIndex:n,moved:!1},(s=(r=this.canvas).setPointerCapture)==null||s.call(r,e.pointerId))}handlePointerMove(e){if(this.dragState.activeIndex<0||!this.lastImageMessage)return;const t=bs({clientX:e.clientX,clientY:e.clientY,rect:this.canvas.getBoundingClientRect(),imageWidth:Number(this.lastImageMessage.width),imageHeight:Number(this.lastImageMessage.height)});this.selectedPoints=Lh(this.selectedPoints,this.dragState.activeIndex,t),this.dragState={...this.dragState,moved:!0},this.draw()}handlePointerUp(){if(this.dragState.activeIndex<0)return;const e=this.dragState.moved;this.dragState={activeIndex:-1,moved:!1},e&&(this.suppressNextCanvasClick=!0,this.notifySelectionChanged())}}class Uv{constructor(e){this.rootElement=e,this.logs=[],this.frontendLogs=[],this.displaySettings=Yh(),this.ui=new qh(e),this.ui.renderShell(),this.panelManager=new Hh,this.panelManager.init(e),e.querySelectorAll(".floating-panel").forEach(n=>{this.panelManager.registerPanel(n)});const t=this.ui.getCanvasRefs();this.workspaceView=new Lv({canvas:t.canvas,overlayCanvas:t.overlayCanvas,onSelectionChanged:n=>{this.ui.renderPointList(n),this.refreshActionState()},onMessage:n=>this.addLog(n,"info")}),this.workspaceView.bindPointerEvents(),this.workspaceView.setDisplaySettings(this.displaySettings),this.ui.setDisplaySettings(this.displaySettings),this.ui.renderPointList([]),this.sceneView=new Pv({container:this.ui.getSceneContainer()}),this.topicLayerController=new Gh({ui:this.ui,sceneView:this.sceneView,callbacks:{onLog:(n,r)=>this.addLog(n,r)}}),this.topicLayerController.init(),this.statusMonitorController=new Ah({onStatusChip:(n,r,s)=>this.ui.setStatusChipState(n,r,s),onBatteryVoltage:n=>this.ui.setBatteryVoltage(n),onLog:(n,r)=>this.addLog(n,r)}),this.rosConnectionController=new bh({onConnectionInfo:(n,r,s)=>{this.ui.setConnectionInfo(n,r,s),this.statusMonitorController.setConnectionState(s,r)},onRosReady:n=>{this.statusMonitorController.start(n.ros),this.refreshActionState()},onRosUnavailable:()=>{var n,r;this.statusMonitorController.stop(),(n=this.legacyCommandController)==null||n.reset(),this.ui.syncControlToggleStates((r=this.legacyCommandController)==null?void 0:r.getToggleStateSnapshot()),this.refreshActionState()},onLog:(n,r)=>this.addLog(n,r),onSystemLog:n=>this.handleSystemLog(n),onBaseImage:n=>{this.workspaceView.setBaseImageMessage(n),this.workspaceView.getSelectedPoints().length===0&&this.ui.setResultMessage("IR 图像已就绪，直接在图上点 4 个角点。")},onSavedWorkspacePayload:n=>{this.workspaceView.setSavedWorkspacePayload(n),this.refreshActionState()},onExecutionOverlay:n=>{this.workspaceView.setExecutionOverlayMessage(n),this.ui.setResultMessage("工作区覆盖层已更新，当前统一显示 /pointAI/result_image_raw。")},onPointCloudImage:(n,r)=>{const s=this.sceneView.setPointCloudImageMessage(n,r);this.topicLayerController.updateStats({[`${n}Count`]:s})},onTiePoints:n=>{const r=this.sceneView.setTiePointsMessage(n);this.topicLayerController.updateStats({tiePointCount:r})},onPlanningMarkers:n=>{const r=this.sceneView.setPlanningMarkersMessage(n);this.topicLayerController.updateStats({planningPointCount:r})},onTfMessage:n=>{this.sceneView.handleTfMessage(n),this.topicLayerController.updateStats({tfFrameCount:this.sceneView.getKnownTransformCount()})}}),this.taskActionController=new Fh({rosConnection:this.rosConnectionController,workspaceView:this.workspaceView,callbacks:{onResultMessage:n=>this.ui.setResultMessage(n),onLog:(n,r)=>this.addLog(n,r)}}),this.legacyCommandController=new th({rosConnection:this.rosConnectionController,callbacks:{onResultMessage:n=>this.ui.setResultMessage(n),onLog:(n,r)=>this.addLog(n,r)}}),this.ui.syncControlToggleStates(this.legacyCommandController.getToggleStateSnapshot())}init(){return this.bindUIEvents(),this.refreshActionState(),this.rosConnectionController.connect(),this}bindUIEvents(){this.ui.onToolbarAction(e=>{this.handleToolbarAction(e)}),this.ui.onTaskAction(e=>{this.taskActionController.handle(e),this.refreshActionState()}),this.ui.onWorkspaceAction(e=>{e==="undo"?this.workspaceView.undoSelection():e==="clear"&&this.workspaceView.clearSelection(),this.refreshActionState()}),this.ui.onDisplaySettingsChange(e=>{this.displaySettings={mode:e.mode,gamma:Number.isFinite(e.gamma)?e.gamma:.85,overlayOpacity:Number.isFinite(e.overlayOpacity)?e.overlayOpacity:.88},jh(this.displaySettings),this.workspaceView.setDisplaySettings(this.displaySettings),this.ui.setDisplaySettings(this.displaySettings)}),this.ui.onSceneControlsChange(e=>{this.topicLayerController.handleSceneControlsChange(e)}),this.ui.onTopicLayerControlsChange(e=>{this.topicLayerController.handleLayerControlsChange(e)}),this.ui.onLegacyCommand(e=>{this.legacyCommandController.handle(e,this.ui.getParameterValues())}),this.ui.onControlToggle(e=>{const t=this.legacyCommandController.handleToggle(e,this.ui.getParameterValues());t&&this.ui.setControlToggleState(e,t)}),this.ui.onClearLogs(()=>{this.logs=[],this.ui.renderLogs(this.logs)})}handleToolbarAction(e){if(e&&e.startsWith("toggle-panel:")){const t=e.split(":")[1];this.ui.togglePanelVisible(t);return}}refreshActionState(){const e=this.rosConnectionController.isReady(),t=this.rosConnectionController.getResources(),n=this.workspaceView.getSelectedPoints(),r=this.workspaceView.getSavedWorkspacePoints();this.ui.setTaskButtonsEnabled({submitQuad:e&&!!(t!=null&&t.workspaceQuadPublisher&&(t!=null&&t.runWorkspaceS2Publisher))&&n.length===4,runSavedS2:e&&!!(t!=null&&t.runWorkspaceS2Publisher)&&r.length===4,scanPlan:e&&!!(t!=null&&t.startPseudoSlamScanActionClient),startExecution:e&&!!(t!=null&&t.executionModeService)&&!!(t!=null&&t.startGlobalWorkActionClient),startExecutionKeepMemory:e&&!!(t!=null&&t.executionModeService)&&!!(t!=null&&t.startGlobalWorkActionClient),runBindPathTest:e&&!!(t!=null&&t.runDirectBindPathTestActionClient)}),this.ui.setWorkspaceButtonsEnabled({undo:n.length>0,clear:n.length>0})}addLog(e,t="info"){const n=new Date().toLocaleTimeString("zh-CN",{hour12:!1});this.frontendLogs=[{timestamp:n,message:e,level:t},...this.frontendLogs].slice(0,80)}handleSystemLog(e){var l;const t=String((e==null?void 0:e.msg)||"");if(!t.startsWith("[stdout]"))return;const n=String((e==null?void 0:e.name)||"").split("/").filter(Boolean).pop()||"unknown",r=t.replace(/^\[stdout\]\s*/,""),s=this.mapRosLogLevel(e==null?void 0:e.level),o=(l=e==null?void 0:e.header)==null?void 0:l.stamp,a=o!=null&&o.secs?new Date(Number(o.secs)*1e3+Math.floor(Number(o.nsecs||0)/1e6)).toLocaleTimeString("zh-CN",{hour12:!1}):new Date().toLocaleTimeString("zh-CN",{hour12:!1});this.logs=[{timestamp:a,message:`[${n}] ${r}`,level:s},...this.logs].slice(0,200),this.ui.renderLogs(this.logs)}mapRosLogLevel(e){return e>=16?"error":e>=8?"warn":e>=2?"info":"success"}}const Dv=document.getElementById("app"),_u=new Uv(Dv);window.tieRobotFrontApp=_u;_u.init();
