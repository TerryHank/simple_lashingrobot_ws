(function(){const e=document.createElement("link").relList;if(e&&e.supports&&e.supports("modulepreload"))return;for(const r of document.querySelectorAll('link[rel="modulepreload"]'))n(r);new MutationObserver(r=>{for(const s of r)if(s.type==="childList")for(const o of s.addedNodes)o.tagName==="LINK"&&o.rel==="modulepreload"&&n(o)}).observe(document,{childList:!0,subtree:!0});function t(r){const s={};return r.integrity&&(s.integrity=r.integrity),r.referrerPolicy&&(s.referrerPolicy=r.referrerPolicy),r.crossOrigin==="use-credentials"?s.credentials="include":r.crossOrigin==="anonymous"?s.credentials="omit":s.credentials="same-origin",s}function n(r){if(r.ep)return;r.ep=!0;const s=t(r);fetch(r.href,s)}})();const Lr={globalX:0,globalY:0,globalZ:10,speed:300,stepX:370,stepY:320,startX:0,startY:0,startZ:0,fixedX:0,fixedY:0,fixedZ:0,fixedTheta:0,fixedSpeed:30},fu={globalX:"全局 X",globalY:"全局 Y",globalZ:"全局 Z",speed:"速度",stepX:"步长 X",stepY:"步长 Y",startX:"起点 X",startY:"起点 Y",startZ:"起点 Z",fixedX:"固定 X",fixedY:"固定 Y",fixedZ:"固定 Z",fixedTheta:"固定角度",fixedSpeed:"固定速度"},sc=[{id:1,name:"启动机器人",topic:"/web/cabin/start",type:"std_msgs/Float32",group:"流程控制"},{id:2,name:"规划作业路径",topic:"/web/cabin/plan_path",type:"geometry_msgs/Pose",group:"流程控制"},{id:3,name:"清除作业路径",topic:"/web/cabin/clear_path",type:"std_msgs/Float32",group:"流程控制"},{id:4,name:"开始全局作业",topic:"/web/cabin/start_global_work",type:"std_msgs/Float32",group:"流程控制"},{id:5,name:"重启机器人",topic:"/web/cabin/restart",type:"std_msgs/Float32",group:"流程控制"},{id:6,name:"末端运动调试",topic:"/web/moduan/moduan_move_debug",type:"geometry_msgs/Pose",group:"调试控制"},{id:7,name:"索驱运动调试",topic:"/web/cabin/cabin_move_debug",type:"geometry_msgs/Pose",group:"调试控制"},{id:8,name:"视觉识别调试",topic:"/web/pointAI/process_image",type:"std_msgs/Float32",group:"视觉调试"},{id:9,name:"定点绑扎调试",topic:"/web/moduan/single_bind",type:"std_msgs/Float32",group:"调试控制"},{id:10,name:"暂停作业",topic:"/web/moduan/interrupt_stop",type:"std_msgs/Float32",group:"末端控制"},{id:11,name:"开启绑扎",topic:"/web/moduan/enb_las",type:"std_msgs/Float32",group:"末端控制"},{id:12,name:"开启(关闭)跳绑",topic:"/web/moduan/send_odd_points",type:"std_msgs/Bool",group:"末端控制"},{id:13,name:"恢复作业",topic:"/web/moduan/hand_sovle_warn",type:"std_msgs/Float32",group:"末端控制"},{id:14,name:"开启(关闭)灯光",topic:"/web/moduan/light",type:"std_msgs/Bool",group:"末端控制"},{id:15,name:"末端回零",topic:"/web/moduan/moduan_move_zero",type:"std_msgs/Float32",group:"末端控制"},{id:16,name:"关闭绑扎",topic:"/web/moduan/enb_las",type:"std_msgs/Float32",group:"末端控制"},{id:17,name:"关闭机器人",topic:"/web/cabin/shutdown",type:"std_msgs/Float32",group:"流程控制"},{id:18,name:"急停作业",topic:"/web/moduan/forced_stop",type:"std_msgs/Float32",group:"末端控制"},{id:19,name:"设置Z固定高度",topic:"/web/pointAI/set_height_threshold",type:"std_msgs/Float32",group:"视觉调试"},{id:20,name:"保存作业路径",topic:"/web/cabin/save_path",type:"std_msgs/Float32",group:"流程控制"},{id:21,name:"保存绑扎数据",topic:"/web/moduan/save_binding_data",type:"std_msgs/Float32",group:"末端控制"},{id:22,name:"修正视觉偏差",topic:"/web/pointAI/set_offset",type:"geometry_msgs/Pose",group:"视觉调试"},{id:23,name:"设置索驱速度",topic:"/web/cabin/set_cabin_speed",type:"std_msgs/Float32",group:"流程控制"},{id:24,name:"设置末端速度",topic:"/web/moduan/set_moduan_speed",type:"std_msgs/Float32",group:"末端控制"}];var ac=typeof globalThis<"u"?globalThis:typeof window<"u"?window:typeof global<"u"?global:typeof self<"u"?self:{};function pu(i){return i&&i.__esModule&&Object.prototype.hasOwnProperty.call(i,"default")?i.default:i}/*
object-assign
(c) Sindre Sorhus
@license MIT
*/var Ca=Object.getOwnPropertySymbols,mu=Object.prototype.hasOwnProperty,gu=Object.prototype.propertyIsEnumerable;function _u(i){if(i==null)throw new TypeError("Object.assign cannot be called with null or undefined");return Object(i)}function vu(){try{if(!Object.assign)return!1;var i=new String("abc");if(i[5]="de",Object.getOwnPropertyNames(i)[0]==="5")return!1;for(var e={},t=0;t<10;t++)e["_"+String.fromCharCode(t)]=t;var n=Object.getOwnPropertyNames(e).map(function(s){return e[s]});if(n.join("")!=="0123456789")return!1;var r={};return"abcdefghijklmnopqrst".split("").forEach(function(s){r[s]=s}),Object.keys(Object.assign({},r)).join("")==="abcdefghijklmnopqrst"}catch{return!1}}var In=vu()?Object.assign:function(i,e){for(var t,n=_u(i),r,s=1;s<arguments.length;s++){t=Object(arguments[s]);for(var o in t)mu.call(t,o)&&(n[o]=t[o]);if(Ca){r=Ca(t);for(var a=0;a<r.length;a++)gu.call(t,r[a])&&(n[r[a]]=t[r[a]])}}return n},Ur={exports:{}},Dr,Ra;function pa(){return Ra||(Ra=1,Dr=function(i,e,t){e.forEach(function(n){var r=t[n];i.prototype[n]=function(s){return s.ros=this,new r(s)}})}),Dr}var Ir,Pa;function oc(){return Pa||(Pa=1,Ir=typeof window<"u"?window.WebSocket:WebSocket),Ir}var Nr,La;function xu(){if(La)return Nr;La=1;var i=arguments[3],e=arguments[4],t=arguments[5],n=JSON.stringify;return Nr=function(r,s){for(var o,a=Object.keys(t),l=0,c=a.length;l<c;l++){var u=a[l],h=t[u].exports;if(h===r||h&&h.default===r){o=u;break}}if(!o){o=Math.floor(Math.pow(16,8)*Math.random()).toString(16);for(var d={},l=0,c=a.length;l<c;l++){var u=a[l];d[u]=u}e[o]=["function(require,module,exports){"+r+"(self); }",d]}var f=Math.floor(Math.pow(16,8)*Math.random()).toString(16),g={};g[o]=o,e[f]=["function(require,module,exports){var f = require("+n(o)+");(f.default ? f.default : f)(self);}",g];var x={};m(f);function m(I){x[I]=!0;for(var P in e[I][1]){var Y=e[I][1][P];x[Y]||m(Y)}}var p="("+i+")({"+Object.keys(x).map(function(I){return n(I)+":["+e[I][0]+","+n(e[I][1])+"]"}).join(",")+"},{},["+n(f)+"])",w=window.URL||window.webkitURL||window.mozURL||window.msURL,S=new Blob([p],{type:"text/javascript"});if(s&&s.bare)return S;var A=w.createObjectURL(S),k=new Worker(A);return k.objectURL=A,k},Nr}var Or={exports:{}},Ua;function yu(){return Ua||(Ua=1,function(i){function e(c){var u={};function h(f){if(u[f])return u[f].exports;var g=u[f]={i:f,l:!1,exports:{}};return c[f].call(g.exports,g,g.exports,h),g.l=!0,g.exports}h.m=c,h.c=u,h.i=function(f){return f},h.d=function(f,g,x){h.o(f,g)||Object.defineProperty(f,g,{configurable:!1,enumerable:!0,get:x})},h.r=function(f){Object.defineProperty(f,"__esModule",{value:!0})},h.n=function(f){var g=f&&f.__esModule?function(){return f.default}:function(){return f};return h.d(g,"a",g),g},h.o=function(f,g){return Object.prototype.hasOwnProperty.call(f,g)},h.p="/",h.oe=function(f){throw console.error(f),f};var d=h(h.s=ENTRY_MODULE);return d.default||d}var t="[\\.|\\-|\\+|\\w|/|@]+",n="\\(\\s*(/\\*.*?\\*/)?\\s*.*?("+t+").*?\\)";function r(c){return(c+"").replace(/[.?*+^$[\]\\(){}|-]/g,"\\$&")}function s(c){return!isNaN(1*c)}function o(c,u,h){var d={};d[h]=[];var f=u.toString(),g=f.match(/^function\s?\w*\(\w+,\s*\w+,\s*(\w+)\)/);if(!g)return d;for(var x=g[1],m=new RegExp("(\\\\n|\\W)"+r(x)+n,"g"),p;p=m.exec(f);)p[3]!=="dll-reference"&&d[h].push(p[3]);for(m=new RegExp("\\("+r(x)+'\\("(dll-reference\\s('+t+'))"\\)\\)'+n,"g");p=m.exec(f);)c[p[2]]||(d[h].push(p[1]),c[p[2]]=__webpack_require__(p[1]).m),d[p[2]]=d[p[2]]||[],d[p[2]].push(p[4]);for(var w=Object.keys(d),S=0;S<w.length;S++)for(var A=0;A<d[w[S]].length;A++)s(d[w[S]][A])&&(d[w[S]][A]=1*d[w[S]][A]);return d}function a(c){var u=Object.keys(c);return u.reduce(function(h,d){return h||c[d].length>0},!1)}function l(c,u){for(var h={main:[u]},d={main:[]},f={main:{}};a(h);)for(var g=Object.keys(h),x=0;x<g.length;x++){var m=g[x],p=h[m],w=p.pop();if(f[m]=f[m]||{},!(f[m][w]||!c[m][w])){f[m][w]=!0,d[m]=d[m]||[],d[m].push(w);for(var S=o(c,c[m][w],m),A=Object.keys(S),k=0;k<A.length;k++)h[A[k]]=h[A[k]]||[],h[A[k]]=h[A[k]].concat(S[A[k]])}}return d}i.exports=function(c,u){u=u||{};var h={main:__webpack_modules__},d=u.all?{main:Object.keys(h.main)}:l(h,c),f="";Object.keys(d).filter(function(w){return w!=="main"}).forEach(function(w){for(var S=0;d[w][S];)S++;d[w].push(S),h[w][S]="(function(module, exports, __webpack_require__) { module.exports = __webpack_require__; })",f=f+"var "+w+" = ("+e.toString().replace("ENTRY_MODULE",JSON.stringify(S))+")({"+d[w].map(function(A){return""+JSON.stringify(A)+": "+h[w][A].toString()}).join(",")+`});
`}),f=f+"new (("+e.toString().replace("ENTRY_MODULE",JSON.stringify(c))+")({"+d.main.map(function(w){return""+JSON.stringify(w)+": "+h.main[w].toString()}).join(",")+"}))(self);";var g=new window.Blob([f],{type:"text/javascript"});if(u.bare)return g;var x=window.URL||window.webkitURL||window.mozURL||window.msURL,m=x.createObjectURL(g),p=new window.Worker(m);return p.objectURL=m,p}}(Or)),Or.exports}var Fr,Da;function Su(){if(Da)return Fr;Da=1;var i=i||oc();return Fr=function(e){var t=null;function n(s){var o=s.data;o instanceof ArrayBuffer?e.postMessage(o,[o]):e.postMessage(o)}function r(s){e.postMessage({type:s.type})}e.addEventListener("message",function(s){var o=s.data;if(typeof o=="string")t.send(o);else if(o.hasOwnProperty("close"))t.close(),t=null;else if(o.hasOwnProperty("uri")){var a=o.uri;t=new i(a),t.binaryType="arraybuffer",t.onmessage=n,t.onclose=r,t.onopen=r,t.onerror=r}else throw"Unknown message to WorkerSocket"})},Fr}var Br,Ia;function Mu(){if(Ia)return Br;Ia=1;try{var i=xu()}catch{var i=yu()}var e=Su();function t(n){this.socket_=i(e),this.socket_.addEventListener("message",this.handleWorkerMessage_.bind(this)),this.socket_.postMessage({uri:n})}return t.prototype.handleWorkerMessage_=function(n){var r=n.data;if(r instanceof ArrayBuffer||typeof r=="string")this.onmessage(n);else{var s=r.type;if(s==="close")this.onclose(null);else if(s==="open")this.onopen(null);else if(s==="error")this.onerror(null);else throw"Unknown message from workersocket"}},t.prototype.send=function(n){this.socket_.postMessage(n)},t.prototype.close=function(){this.socket_.postMessage({close:!0})},Br=t,Br}var kr,Na;function bu(){return Na||(Na=1,kr=function(){return document.createElement("canvas")}),kr}var zr,Oa;function Eu(){if(Oa)return zr;Oa=1;var i=bu(),e=i.Image||window.Image;function t(n,r){var s=new e;s.onload=function(){var o=new i,a=o.getContext("2d");o.width=s.width,o.height=s.height,a.imageSmoothingEnabled=!1,a.webkitImageSmoothingEnabled=!1,a.mozImageSmoothingEnabled=!1,a.drawImage(s,0,0);for(var l=a.getImageData(0,0,s.width,s.height).data,c="",u=0;u<l.length;u+=4)c+=String.fromCharCode(l[u],l[u+1],l[u+2]);r(JSON.parse(c))},s.src="data:image/png;base64,"+n}return zr=t,zr}var Vr={exports:{}},Fa;function wu(){return Fa||(Fa=1,function(i){(function(e,t){var n=Math.pow(2,-24),r=Math.pow(2,32),s=Math.pow(2,53);function o(c){var u=new ArrayBuffer(256),h=new DataView(u),d,f=0;function g(C){for(var H=u.byteLength,O=f+C;H<O;)H*=2;if(H!==u.byteLength){var $=h;u=new ArrayBuffer(H),h=new DataView(u);for(var z=f+3>>2,q=0;q<z;++q)h.setUint32(q*4,$.getUint32(q*4))}return d=C,h}function x(){f+=d}function m(C){x(g(8).setFloat64(f,C))}function p(C){x(g(1).setUint8(f,C))}function w(C){for(var H=g(C.length),O=0;O<C.length;++O)H.setUint8(f+O,C[O]);x()}function S(C){x(g(2).setUint16(f,C))}function A(C){x(g(4).setUint32(f,C))}function k(C){var H=C%r,O=(C-H)/r,$=g(8);$.setUint32(f,O),$.setUint32(f+4,H),x()}function I(C,H){H<24?p(C<<5|H):H<256?(p(C<<5|24),p(H)):H<65536?(p(C<<5|25),S(H)):H<4294967296?(p(C<<5|26),A(H)):(p(C<<5|27),k(H))}function P(C){var H;if(C===!1)return p(244);if(C===!0)return p(245);if(C===null)return p(246);if(C===t)return p(247);switch(typeof C){case"number":if(Math.floor(C)===C){if(0<=C&&C<=s)return I(0,C);if(-s<=C&&C<0)return I(1,-(C+1))}return p(251),m(C);case"string":var O=[];for(H=0;H<C.length;++H){var $=C.charCodeAt(H);$<128?O.push($):$<2048?(O.push(192|$>>6),O.push(128|$&63)):$<55296?(O.push(224|$>>12),O.push(128|$>>6&63),O.push(128|$&63)):($=($&1023)<<10,$|=C.charCodeAt(++H)&1023,$+=65536,O.push(240|$>>18),O.push(128|$>>12&63),O.push(128|$>>6&63),O.push(128|$&63))}return I(3,O.length),w(O);default:var z;if(Array.isArray(C))for(z=C.length,I(4,z),H=0;H<z;++H)P(C[H]);else if(C instanceof Uint8Array)I(2,C.length),w(C);else{var q=Object.keys(C);for(z=q.length,I(5,z),H=0;H<z;++H){var te=q[H];P(te),P(C[te])}}}}if(P(c),"slice"in u)return u.slice(0,f);for(var Y=new ArrayBuffer(f),E=new DataView(Y),b=0;b<f;++b)E.setUint8(b,h.getUint8(b));return Y}function a(c,u,h){var d=new DataView(c),f=0;typeof u!="function"&&(u=function(O){return O}),typeof h!="function"&&(h=function(){return t});function g(O,$){return f+=$,O}function x(O){return g(new Uint8Array(c,f,O),O)}function m(){var O=new ArrayBuffer(4),$=new DataView(O),z=A(),q=z&32768,te=z&31744,v=z&1023;if(te===31744)te=261120;else if(te!==0)te+=114688;else if(v!==0)return v*n;return $.setUint32(0,q<<16|te<<13|v<<13),$.getFloat32(0)}function p(){return g(d.getFloat32(f),4)}function w(){return g(d.getFloat64(f),8)}function S(){return g(d.getUint8(f),1)}function A(){return g(d.getUint16(f),2)}function k(){return g(d.getUint32(f),4)}function I(){return k()*r+k()}function P(){return d.getUint8(f)!==255?!1:(f+=1,!0)}function Y(O){if(O<24)return O;if(O===24)return S();if(O===25)return A();if(O===26)return k();if(O===27)return I();if(O===31)return-1;throw"Invalid length encoding"}function E(O){var $=S();if($===255)return-1;var z=Y($&31);if(z<0||$>>5!==O)throw"Invalid indefinite length element";return z}function b(O,$){for(var z=0;z<$;++z){var q=S();q&128&&(q<224?(q=(q&31)<<6|S()&63,$-=1):q<240?(q=(q&15)<<12|(S()&63)<<6|S()&63,$-=2):(q=(q&15)<<18|(S()&63)<<12|(S()&63)<<6|S()&63,$-=3)),q<65536?O.push(q):(q-=65536,O.push(55296|q>>10),O.push(56320|q&1023))}}function C(){var O=S(),$=O>>5,z=O&31,q,te;if($===7)switch(z){case 25:return m();case 26:return p();case 27:return w()}if(te=Y(z),te<0&&($<2||6<$))throw"Invalid length";switch($){case 0:return te;case 1:return-1-te;case 2:if(te<0){for(var v=[],T=0;(te=E($))>=0;)T+=te,v.push(x(te));var U=new Uint8Array(T),N=0;for(q=0;q<v.length;++q)U.set(v[q],N),N+=v[q].length;return U}return x(te);case 3:var V=[];if(te<0)for(;(te=E($))>=0;)b(V,te);else b(V,te);return String.fromCharCode.apply(null,V);case 4:var K;if(te<0)for(K=[];!P();)K.push(C());else for(K=new Array(te),q=0;q<te;++q)K[q]=C();return K;case 5:var D={};for(q=0;q<te||te<0&&!P();++q){var B=C();D[B]=C()}return D;case 6:return u(C(),te);case 7:switch(te){case 20:return!1;case 21:return!0;case 22:return null;case 23:return t;default:return h(te)}}}var H=C();if(f!==c.byteLength)throw"Remaining bytes";return H}var l={encode:o,decode:a};i.exports?i.exports=l:e.CBOR||(e.CBOR=l)})(ac)}(Vr)),Vr.exports}var Gr={exports:{}},Ba;function Tu(){return Ba||(Ba=1,function(i){var e=Math.pow(2,32),t=!1;function n(){t||(t=!0,console.warn("CBOR 64-bit integer array values may lose precision. No further warnings."))}function r(u){n();for(var h=u.byteLength,d=u.byteOffset,f=h/8,g=u.buffer.slice(d,d+h),x=new Uint32Array(g),m=new Array(f),p=0;p<f;p++){var w=p*2,S=x[w],A=x[w+1];m[p]=S+e*A}return m}function s(u){n();for(var h=u.byteLength,d=u.byteOffset,f=h/8,g=u.buffer.slice(d,d+h),x=new Uint32Array(g),m=new Int32Array(g),p=new Array(f),w=0;w<f;w++){var S=w*2,A=x[S],k=m[S+1];p[w]=A+e*k}return p}function o(u,h){var d=u.byteLength,f=u.byteOffset,g=u.buffer.slice(f,f+d);return new h(g)}var a={64:Uint8Array,69:Uint16Array,70:Uint32Array,72:Int8Array,77:Int16Array,78:Int32Array,85:Float32Array,86:Float64Array},l={71:r,79:s};function c(u,h){if(h in a){var d=a[h];return o(u,d)}return h in l?l[h](u):u}i.exports&&(i.exports=c)}(Gr)),Gr.exports}var Hr,ka;function Au(){if(ka)return Hr;ka=1;var i=Eu(),e=wu(),t=Tu(),n=null;typeof bson<"u"&&(n=bson().BSON);function r(s){var o=null;s.transportOptions.decoder&&(o=s.transportOptions.decoder);function a(u){u.op==="publish"?s.emit(u.topic,u.msg):u.op==="service_response"?s.emit(u.id,u):u.op==="call_service"?s.emit(u.service,u):u.op==="status"&&(u.id?s.emit("status:"+u.id,u):s.emit("status",u))}function l(u,h){u.op==="png"?i(u.data,h):h(u)}function c(u,h){if(!n)throw"Cannot process BSON encoded message without BSON header.";var d=new FileReader;d.onload=function(){var f=new Uint8Array(this.result),g=n.deserialize(f);h(g)},d.readAsArrayBuffer(u)}return{onopen:function(h){s.isConnected=!0,s.emit("connection",h)},onclose:function(h){s.isConnected=!1,s.emit("close",h)},onerror:function(h){s.emit("error",h)},onmessage:function(h){if(o)o(h.data,function(g){a(g)});else if(typeof Blob<"u"&&h.data instanceof Blob)c(h.data,function(g){l(g,a)});else if(h.data instanceof ArrayBuffer){var d=e.decode(h.data,t);a(d)}else{var f=JSON.parse(typeof h=="string"?h:h.data);l(f,a)}}}}return Hr=r,Hr}var Wr,za;function lc(){if(za)return Wr;za=1;var i=In;function e(t){i(this,t)}return Wr=e,Wr}var Xr,Va;function wi(){if(Va)return Xr;Va=1;var i=In;function e(t){i(this,t)}return Xr=e,Xr}var qr={exports:{}};/*!
 * EventEmitter2
 * https://github.com/hij1nx/EventEmitter2
 *
 * Copyright (c) 2013 hij1nx
 * Licensed under the MIT license.
 */var Ga;function Nn(){return Ga||(Ga=1,function(i,e){(function(t){var n=Object.hasOwnProperty,r=Array.isArray?Array.isArray:function(T){return Object.prototype.toString.call(T)==="[object Array]"},s=10,o=typeof process=="object"&&typeof process.nextTick=="function",a=typeof Symbol=="function",l=typeof Reflect=="object",c=typeof setImmediate=="function",u=c?setImmediate:setTimeout,h=a?l&&typeof Reflect.ownKeys=="function"?Reflect.ownKeys:function(v){var T=Object.getOwnPropertyNames(v);return T.push.apply(T,Object.getOwnPropertySymbols(v)),T}:Object.keys;function d(){this._events={},this._conf&&f.call(this,this._conf)}function f(v){v&&(this._conf=v,v.delimiter&&(this.delimiter=v.delimiter),v.maxListeners!==t&&(this._maxListeners=v.maxListeners),v.wildcard&&(this.wildcard=v.wildcard),v.newListener&&(this._newListener=v.newListener),v.removeListener&&(this._removeListener=v.removeListener),v.verboseMemoryLeak&&(this.verboseMemoryLeak=v.verboseMemoryLeak),v.ignoreErrors&&(this.ignoreErrors=v.ignoreErrors),this.wildcard&&(this.listenerTree={}))}function g(v,T){var U="(node) warning: possible EventEmitter memory leak detected. "+v+" listeners added. Use emitter.setMaxListeners() to increase limit.";if(this.verboseMemoryLeak&&(U+=" Event name: "+T+"."),typeof process<"u"&&process.emitWarning){var N=new Error(U);N.name="MaxListenersExceededWarning",N.emitter=this,N.count=v,process.emitWarning(N)}else console.error(U),console.trace&&console.trace()}var x=function(v,T,U){var N=arguments.length;switch(N){case 0:return[];case 1:return[v];case 2:return[v,T];case 3:return[v,T,U];default:for(var V=new Array(N);N--;)V[N]=arguments[N];return V}};function m(v,T){for(var U={},N,V=v.length,K=0,D=0;D<V;D++)N=v[D],U[N]=D<K?T[D]:t;return U}function p(v,T,U){this._emitter=v,this._target=T,this._listeners={},this._listenersCount=0;var N,V;if((U.on||U.off)&&(N=U.on,V=U.off),T.addEventListener?(N=T.addEventListener,V=T.removeEventListener):T.addListener?(N=T.addListener,V=T.removeListener):T.on&&(N=T.on,V=T.off),!N&&!V)throw Error("target does not implement any known event API");if(typeof N!="function")throw TypeError("on method must be a function");if(typeof V!="function")throw TypeError("off method must be a function");this._on=N,this._off=V;var K=v._observers;K?K.push(this):v._observers=[this]}Object.assign(p.prototype,{subscribe:function(v,T,U){var N=this,V=this._target,K=this._emitter,D=this._listeners,B=function(){var j=x.apply(null,arguments),X={data:j,name:T,original:v};if(U){var le=U.call(V,X);le!==!1&&K.emit.apply(K,[X.name].concat(j));return}K.emit.apply(K,[T].concat(j))};if(D[v])throw Error("Event '"+v+"' is already listening");this._listenersCount++,K._newListener&&K._removeListener&&!N._onNewListener?(this._onNewListener=function(j){j===T&&D[v]===null&&(D[v]=B,N._on.call(V,v,B))},K.on("newListener",this._onNewListener),this._onRemoveListener=function(j){j===T&&!K.hasListeners(j)&&D[v]&&(D[v]=null,N._off.call(V,v,B))},D[v]=null,K.on("removeListener",this._onRemoveListener)):(D[v]=B,N._on.call(V,v,B))},unsubscribe:function(v){var T=this,U=this._listeners,N=this._emitter,V,K,D=this._off,B=this._target,j;if(v&&typeof v!="string")throw TypeError("event must be a string");function X(){T._onNewListener&&(N.off("newListener",T._onNewListener),N.off("removeListener",T._onRemoveListener),T._onNewListener=null,T._onRemoveListener=null);var le=Y.call(N,T);N._observers.splice(le,1)}if(v){if(V=U[v],!V)return;D.call(B,v,V),delete U[v],--this._listenersCount||X()}else{for(K=h(U),j=K.length;j-- >0;)v=K[j],D.call(B,v,U[v]);this._listeners={},this._listenersCount=0,X()}}});function w(v,T,U,N){var V=Object.assign({},T);if(!v)return V;if(typeof v!="object")throw TypeError("options must be an object");var K=Object.keys(v),D=K.length,B,j,X;function le(ge){throw Error('Invalid "'+B+'" option value'+(ge?". Reason: "+ge:""))}for(var ve=0;ve<D;ve++){if(B=K[ve],!n.call(T,B))throw Error('Unknown "'+B+'" option');j=v[B],j!==t&&(X=U[B],V[B]=X?X(j,le):j)}return V}function S(v,T){return(typeof v!="function"||!v.hasOwnProperty("prototype"))&&T("value must be a constructor"),v}function A(v){var T="value must be type of "+v.join("|"),U=v.length,N=v[0],V=v[1];return U===1?function(K,D){if(typeof K===N)return K;D(T)}:U===2?function(K,D){var B=typeof K;if(B===N||B===V)return K;D(T)}:function(K,D){for(var B=typeof K,j=U;j-- >0;)if(B===v[j])return K;D(T)}}var k=A(["function"]),I=A(["object","function"]);function P(v,T,U){var N,V,K=0,D,B=new v(function(j,X,le){U=w(U,{timeout:0,overload:!1},{timeout:function(xe,we){return xe*=1,(typeof xe!="number"||xe<0||!Number.isFinite(xe))&&we("timeout must be a positive number"),xe}}),N=!U.overload&&typeof v.prototype.cancel=="function"&&typeof le=="function";function ve(){V&&(V=null),K&&(clearTimeout(K),K=0)}var ge=function(xe){ve(),j(xe)},L=function(xe){ve(),X(xe)};N?T(ge,L,le):(V=[function(xe){L(xe||Error("canceled"))}],T(ge,L,function(xe){if(D)throw Error("Unable to subscribe on cancel event asynchronously");if(typeof xe!="function")throw TypeError("onCancel callback must be a function");V.push(xe)}),D=!0),U.timeout>0&&(K=setTimeout(function(){var xe=Error("timeout");xe.code="ETIMEDOUT",K=0,B.cancel(xe),X(xe)},U.timeout))});return N||(B.cancel=function(j){if(V){for(var X=V.length,le=1;le<X;le++)V[le](j);V[0](j),V=null}}),B}function Y(v){var T=this._observers;if(!T)return-1;for(var U=T.length,N=0;N<U;N++)if(T[N]._target===v)return N;return-1}function E(v,T,U,N,V){if(!U)return null;if(N===0){var K=typeof T;if(K==="string"){var D,B,j=0,X=0,le=this.delimiter,ve=le.length;if((B=T.indexOf(le))!==-1){D=new Array(5);do D[j++]=T.slice(X,B),X=B+ve;while((B=T.indexOf(le,X))!==-1);D[j++]=T.slice(X),T=D,V=j}else T=[T],V=1}else K==="object"?V=T.length:(T=[T],V=1)}var ge=null,L,xe,we,Xe,Me,Be=T[N],Ie=T[N+1],Ae,Ve;if(N===V)U._listeners&&(typeof U._listeners=="function"?(v&&v.push(U._listeners),ge=[U]):(v&&v.push.apply(v,U._listeners),ge=[U]));else if(Be==="*"){for(Ae=h(U),B=Ae.length;B-- >0;)L=Ae[B],L!=="_listeners"&&(Ve=E(v,T,U[L],N+1,V),Ve&&(ge?ge.push.apply(ge,Ve):ge=Ve));return ge}else if(Be==="**"){for(Me=N+1===V||N+2===V&&Ie==="*",Me&&U._listeners&&(ge=E(v,T,U,V,V)),Ae=h(U),B=Ae.length;B-- >0;)L=Ae[B],L!=="_listeners"&&(L==="*"||L==="**"?(U[L]._listeners&&!Me&&(Ve=E(v,T,U[L],V,V),Ve&&(ge?ge.push.apply(ge,Ve):ge=Ve)),Ve=E(v,T,U[L],N,V)):L===Ie?Ve=E(v,T,U[L],N+2,V):Ve=E(v,T,U[L],N,V),Ve&&(ge?ge.push.apply(ge,Ve):ge=Ve));return ge}else U[Be]&&(ge=E(v,T,U[Be],N+1,V));if(xe=U["*"],xe&&E(v,T,xe,N+1,V),we=U["**"],we)if(N<V)for(we._listeners&&E(v,T,we,V,V),Ae=h(we),B=Ae.length;B-- >0;)L=Ae[B],L!=="_listeners"&&(L===Ie?E(v,T,we[L],N+2,V):L===Be?E(v,T,we[L],N+1,V):(Xe={},Xe[L]=we[L],E(v,T,{"**":Xe},N+1,V)));else we._listeners?E(v,T,we,V,V):we["*"]&&we["*"]._listeners&&E(v,T,we["*"],V,V);return ge}function b(v,T,U){var N=0,V=0,K,D=this.delimiter,B=D.length,j;if(typeof v=="string")if((K=v.indexOf(D))!==-1){j=new Array(5);do j[N++]=v.slice(V,K),V=K+B;while((K=v.indexOf(D,V))!==-1);j[N++]=v.slice(V)}else j=[v],N=1;else j=v,N=v.length;if(N>1){for(K=0;K+1<N;K++)if(j[K]==="**"&&j[K+1]==="**")return}var X=this.listenerTree,le;for(K=0;K<N;K++)if(le=j[K],X=X[le]||(X[le]={}),K===N-1)return X._listeners?(typeof X._listeners=="function"&&(X._listeners=[X._listeners]),U?X._listeners.unshift(T):X._listeners.push(T),!X._listeners.warned&&this._maxListeners>0&&X._listeners.length>this._maxListeners&&(X._listeners.warned=!0,g.call(this,X._listeners.length,le))):X._listeners=T,!0;return!0}function C(v,T,U,N){for(var V=h(v),K=V.length,D,B,j,X=v._listeners,le;K-- >0;)B=V[K],D=v[B],B==="_listeners"?j=U:j=U?U.concat(B):[B],le=N||typeof B=="symbol",X&&T.push(le?j:j.join(this.delimiter)),typeof D=="object"&&C.call(this,D,T,j,le);return T}function H(v){for(var T=h(v),U=T.length,N,V,K;U-- >0;)V=T[U],N=v[V],N&&(K=!0,V!=="_listeners"&&!H(N)&&delete v[V]);return K}function O(v,T,U){this.emitter=v,this.event=T,this.listener=U}O.prototype.off=function(){return this.emitter.off(this.event,this.listener),this};function $(v,T,U){if(U===!0)V=!0;else if(U===!1)N=!0;else{if(!U||typeof U!="object")throw TypeError("options should be an object or true");var N=U.async,V=U.promisify,K=U.nextTick,D=U.objectify}if(N||K||V){var B=T,j=T._origin||T;if(K&&!o)throw Error("process.nextTick is not supported");V===t&&(V=T.constructor.name==="AsyncFunction"),T=function(){var X=arguments,le=this,ve=this.event;return V?K?Promise.resolve():new Promise(function(ge){u(ge)}).then(function(){return le.event=ve,B.apply(le,X)}):(K?process.nextTick:u)(function(){le.event=ve,B.apply(le,X)})},T._async=!0,T._origin=j}return[T,D?new O(this,v,T):this]}function z(v){this._events={},this._newListener=!1,this._removeListener=!1,this.verboseMemoryLeak=!1,f.call(this,v)}z.EventEmitter2=z,z.prototype.listenTo=function(v,T,U){if(typeof v!="object")throw TypeError("target musts be an object");var N=this;U=w(U,{on:t,off:t,reducers:t},{on:k,off:k,reducers:I});function V(K){if(typeof K!="object")throw TypeError("events must be an object");var D=U.reducers,B=Y.call(N,v),j;B===-1?j=new p(N,v,U):j=N._observers[B];for(var X=h(K),le=X.length,ve,ge=typeof D=="function",L=0;L<le;L++)ve=X[L],j.subscribe(ve,K[ve]||ve,ge?D:D&&D[ve])}return r(T)?V(m(T)):V(typeof T=="string"?m(T.split(/\s+/)):T),this},z.prototype.stopListeningTo=function(v,T){var U=this._observers;if(!U)return!1;var N=U.length,V,K=!1;if(v&&typeof v!="object")throw TypeError("target should be an object");for(;N-- >0;)V=U[N],(!v||V._target===v)&&(V.unsubscribe(T),K=!0);return K},z.prototype.delimiter=".",z.prototype.setMaxListeners=function(v){v!==t&&(this._maxListeners=v,this._conf||(this._conf={}),this._conf.maxListeners=v)},z.prototype.getMaxListeners=function(){return this._maxListeners},z.prototype.event="",z.prototype.once=function(v,T,U){return this._once(v,T,!1,U)},z.prototype.prependOnceListener=function(v,T,U){return this._once(v,T,!0,U)},z.prototype._once=function(v,T,U,N){return this._many(v,1,T,U,N)},z.prototype.many=function(v,T,U,N){return this._many(v,T,U,!1,N)},z.prototype.prependMany=function(v,T,U,N){return this._many(v,T,U,!0,N)},z.prototype._many=function(v,T,U,N,V){var K=this;if(typeof U!="function")throw new Error("many only accepts instances of Function");function D(){return--T===0&&K.off(v,D),U.apply(this,arguments)}return D._origin=U,this._on(v,D,N,V)},z.prototype.emit=function(){if(!this._events&&!this._all)return!1;this._events||d.call(this);var v=arguments[0],T,U=this.wildcard,N,V,K,D,B;if(v==="newListener"&&!this._newListener&&!this._events.newListener)return!1;if(U&&(T=v,v!=="newListener"&&v!=="removeListener"&&typeof v=="object")){if(V=v.length,a){for(K=0;K<V;K++)if(typeof v[K]=="symbol"){B=!0;break}}B||(v=v.join(this.delimiter))}var j=arguments.length,X;if(this._all&&this._all.length)for(X=this._all.slice(),K=0,V=X.length;K<V;K++)switch(this.event=v,j){case 1:X[K].call(this,v);break;case 2:X[K].call(this,v,arguments[1]);break;case 3:X[K].call(this,v,arguments[1],arguments[2]);break;default:X[K].apply(this,arguments)}if(U)X=[],E.call(this,X,T,this.listenerTree,0,V);else if(X=this._events[v],typeof X=="function"){switch(this.event=v,j){case 1:X.call(this);break;case 2:X.call(this,arguments[1]);break;case 3:X.call(this,arguments[1],arguments[2]);break;default:for(N=new Array(j-1),D=1;D<j;D++)N[D-1]=arguments[D];X.apply(this,N)}return!0}else X&&(X=X.slice());if(X&&X.length){if(j>3)for(N=new Array(j-1),D=1;D<j;D++)N[D-1]=arguments[D];for(K=0,V=X.length;K<V;K++)switch(this.event=v,j){case 1:X[K].call(this);break;case 2:X[K].call(this,arguments[1]);break;case 3:X[K].call(this,arguments[1],arguments[2]);break;default:X[K].apply(this,N)}return!0}else if(!this.ignoreErrors&&!this._all&&v==="error")throw arguments[1]instanceof Error?arguments[1]:new Error("Uncaught, unspecified 'error' event.");return!!this._all},z.prototype.emitAsync=function(){if(!this._events&&!this._all)return!1;this._events||d.call(this);var v=arguments[0],T=this.wildcard,U,N,V,K,D,B;if(v==="newListener"&&!this._newListener&&!this._events.newListener)return Promise.resolve([!1]);if(T&&(U=v,v!=="newListener"&&v!=="removeListener"&&typeof v=="object")){if(K=v.length,a){for(D=0;D<K;D++)if(typeof v[D]=="symbol"){N=!0;break}}N||(v=v.join(this.delimiter))}var j=[],X=arguments.length,le;if(this._all)for(D=0,K=this._all.length;D<K;D++)switch(this.event=v,X){case 1:j.push(this._all[D].call(this,v));break;case 2:j.push(this._all[D].call(this,v,arguments[1]));break;case 3:j.push(this._all[D].call(this,v,arguments[1],arguments[2]));break;default:j.push(this._all[D].apply(this,arguments))}if(T?(le=[],E.call(this,le,U,this.listenerTree,0)):le=this._events[v],typeof le=="function")switch(this.event=v,X){case 1:j.push(le.call(this));break;case 2:j.push(le.call(this,arguments[1]));break;case 3:j.push(le.call(this,arguments[1],arguments[2]));break;default:for(V=new Array(X-1),B=1;B<X;B++)V[B-1]=arguments[B];j.push(le.apply(this,V))}else if(le&&le.length){if(le=le.slice(),X>3)for(V=new Array(X-1),B=1;B<X;B++)V[B-1]=arguments[B];for(D=0,K=le.length;D<K;D++)switch(this.event=v,X){case 1:j.push(le[D].call(this));break;case 2:j.push(le[D].call(this,arguments[1]));break;case 3:j.push(le[D].call(this,arguments[1],arguments[2]));break;default:j.push(le[D].apply(this,V))}}else if(!this.ignoreErrors&&!this._all&&v==="error")return arguments[1]instanceof Error?Promise.reject(arguments[1]):Promise.reject("Uncaught, unspecified 'error' event.");return Promise.all(j)},z.prototype.on=function(v,T,U){return this._on(v,T,!1,U)},z.prototype.prependListener=function(v,T,U){return this._on(v,T,!0,U)},z.prototype.onAny=function(v){return this._onAny(v,!1)},z.prototype.prependAny=function(v){return this._onAny(v,!0)},z.prototype.addListener=z.prototype.on,z.prototype._onAny=function(v,T){if(typeof v!="function")throw new Error("onAny only accepts instances of Function");return this._all||(this._all=[]),T?this._all.unshift(v):this._all.push(v),this},z.prototype._on=function(v,T,U,N){if(typeof v=="function")return this._onAny(v,T),this;if(typeof T!="function")throw new Error("on only accepts instances of Function");this._events||d.call(this);var V=this,K;return N!==t&&(K=$.call(this,v,T,N),T=K[0],V=K[1]),this._newListener&&this.emit("newListener",v,T),this.wildcard?(b.call(this,v,T,U),V):(this._events[v]?(typeof this._events[v]=="function"&&(this._events[v]=[this._events[v]]),U?this._events[v].unshift(T):this._events[v].push(T),!this._events[v].warned&&this._maxListeners>0&&this._events[v].length>this._maxListeners&&(this._events[v].warned=!0,g.call(this,this._events[v].length,v))):this._events[v]=T,V)},z.prototype.off=function(v,T){if(typeof T!="function")throw new Error("removeListener only takes instances of Function");var U,N=[];if(this.wildcard){var V=typeof v=="string"?v.split(this.delimiter):v.slice();if(N=E.call(this,null,V,this.listenerTree,0),!N)return this}else{if(!this._events[v])return this;U=this._events[v],N.push({_listeners:U})}for(var K=0;K<N.length;K++){var D=N[K];if(U=D._listeners,r(U)){for(var B=-1,j=0,X=U.length;j<X;j++)if(U[j]===T||U[j].listener&&U[j].listener===T||U[j]._origin&&U[j]._origin===T){B=j;break}if(B<0)continue;return this.wildcard?D._listeners.splice(B,1):this._events[v].splice(B,1),U.length===0&&(this.wildcard?delete D._listeners:delete this._events[v]),this._removeListener&&this.emit("removeListener",v,T),this}else(U===T||U.listener&&U.listener===T||U._origin&&U._origin===T)&&(this.wildcard?delete D._listeners:delete this._events[v],this._removeListener&&this.emit("removeListener",v,T))}return this.listenerTree&&H(this.listenerTree),this},z.prototype.offAny=function(v){var T=0,U=0,N;if(v&&this._all&&this._all.length>0){for(N=this._all,T=0,U=N.length;T<U;T++)if(v===N[T])return N.splice(T,1),this._removeListener&&this.emit("removeListenerAny",v),this}else{if(N=this._all,this._removeListener)for(T=0,U=N.length;T<U;T++)this.emit("removeListenerAny",N[T]);this._all=[]}return this},z.prototype.removeListener=z.prototype.off,z.prototype.removeAllListeners=function(v){if(v===t)return!this._events||d.call(this),this;if(this.wildcard){var T=E.call(this,null,v,this.listenerTree,0),U,N;if(!T)return this;for(N=0;N<T.length;N++)U=T[N],U._listeners=null;this.listenerTree&&H(this.listenerTree)}else this._events&&(this._events[v]=null);return this},z.prototype.listeners=function(v){var T=this._events,U,N,V,K,D;if(v===t){if(this.wildcard)throw Error("event name required for wildcard emitter");if(!T)return[];for(U=h(T),K=U.length,V=[];K-- >0;)N=T[U[K]],typeof N=="function"?V.push(N):V.push.apply(V,N);return V}else{if(this.wildcard){if(D=this.listenerTree,!D)return[];var B=[],j=typeof v=="string"?v.split(this.delimiter):v.slice();return E.call(this,B,j,D,0),B}return T?(N=T[v],N?typeof N=="function"?[N]:N:[]):[]}},z.prototype.eventNames=function(v){var T=this._events;return this.wildcard?C.call(this,this.listenerTree,[],null,v):T?h(T):[]},z.prototype.listenerCount=function(v){return this.listeners(v).length},z.prototype.hasListeners=function(v){if(this.wildcard){var T=[],U=typeof v=="string"?v.split(this.delimiter):v.slice();return E.call(this,T,U,this.listenerTree,0),T.length>0}var N=this._events,V=this._all;return!!(V&&V.length||N&&(v===t?h(N).length:N[v]))},z.prototype.listenersAny=function(){return this._all?this._all:[]},z.prototype.waitFor=function(v,T){var U=this,N=typeof T;return N==="number"?T={timeout:T}:N==="function"&&(T={filter:T}),T=w(T,{timeout:0,filter:t,handleError:!1,Promise,overload:!1},{filter:k,Promise:S}),P(T.Promise,function(V,K,D){function B(){var j=T.filter;if(!(j&&!j.apply(U,arguments)))if(U.off(v,B),T.handleError){var X=arguments[0];X?K(X):V(x.apply(null,arguments).slice(1))}else V(x.apply(null,arguments))}D(function(){U.off(v,B)}),U._on(v,B,!1)},{timeout:T.timeout,overload:T.overload})};function q(v,T,U){U=w(U,{Promise,timeout:0,overload:!1},{Promise:S});var N=U.Promise;return P(N,function(V,K,D){var B;if(typeof v.addEventListener=="function"){B=function(){V(x.apply(null,arguments))},D(function(){v.removeEventListener(T,B)}),v.addEventListener(T,B,{once:!0});return}var j=function(){X&&v.removeListener("error",X),V(x.apply(null,arguments))},X;T!=="error"&&(X=function(le){v.removeListener(T,j),K(le)},v.once("error",X)),D(function(){X&&v.removeListener("error",X),v.removeListener(T,j)}),v.once(T,j)},{timeout:U.timeout,overload:U.overload})}var te=z.prototype;Object.defineProperties(z,{defaultMaxListeners:{get:function(){return te._maxListeners},set:function(v){if(typeof v!="number"||v<0||Number.isNaN(v))throw TypeError("n must be a non-negative number");te._maxListeners=v},enumerable:!0},once:{value:q,writable:!0,configurable:!0}}),Object.defineProperties(te,{_maxListeners:{value:s,writable:!0,configurable:!0},_observers:{value:null,writable:!0,configurable:!0}}),i.exports=z})()}(qr)),qr.exports}var Yr,Ha;function Sr(){if(Ha)return Yr;Ha=1;var i=lc();wi();var e=Nn().EventEmitter2;function t(n){n=n||{},this.ros=n.ros,this.name=n.name,this.serviceType=n.serviceType,this.isAdvertised=!1,this._serviceCallback=null}return t.prototype.__proto__=e.prototype,t.prototype.callService=function(n,r,s){if(!this.isAdvertised){var o="call_service:"+this.name+":"+ ++this.ros.idCounter;(r||s)&&this.ros.once(o,function(l){l.result!==void 0&&l.result===!1?typeof s=="function"&&s(l.values):typeof r=="function"&&r(new i(l.values))});var a={op:"call_service",id:o,service:this.name,type:this.serviceType,args:n};this.ros.callOnConnection(a)}},t.prototype.advertise=function(n){this.isAdvertised||typeof n!="function"||(this._serviceCallback=n,this.ros.on(this.name,this._serviceResponse.bind(this)),this.ros.callOnConnection({op:"advertise_service",type:this.serviceType,service:this.name}),this.isAdvertised=!0)},t.prototype.unadvertise=function(){this.isAdvertised&&(this.ros.callOnConnection({op:"unadvertise_service",service:this.name}),this.isAdvertised=!1)},t.prototype._serviceResponse=function(n){var r={},s=this._serviceCallback(n.args,r),o={op:"service_response",service:this.name,values:new i(r),result:s};n.id&&(o.id=n.id),this.ros.callOnConnection(o)},Yr=t,Yr}var jr,Wa;function ma(){if(Wa)return jr;Wa=1;var i=oc(),e=Mu(),t=Au(),n=Sr(),r=wi(),s=In,o=Nn().EventEmitter2;function a(l){l=l||{};var c=this;this.socket=null,this.idCounter=0,this.isConnected=!1,this.transportLibrary=l.transportLibrary||"websocket",this.transportOptions=l.transportOptions||{},this._sendFunc=function(u){c.sendEncodedMessage(u)},typeof l.groovyCompatibility>"u"?this.groovyCompatibility=!0:this.groovyCompatibility=l.groovyCompatibility,this.setMaxListeners(0),l.url&&this.connect(l.url)}return a.prototype.__proto__=o.prototype,a.prototype.connect=function(l){if(this.transportLibrary==="socket.io")this.socket=s(io(l,{"force new connection":!0}),t(this)),this.socket.on("connect",this.socket.onopen),this.socket.on("data",this.socket.onmessage),this.socket.on("close",this.socket.onclose),this.socket.on("error",this.socket.onerror);else if(this.transportLibrary.constructor.name==="RTCPeerConnection")this.socket=s(this.transportLibrary.createDataChannel(l,this.transportOptions),t(this));else if(this.transportLibrary==="websocket"){if(!this.socket||this.socket.readyState===i.CLOSED){var c=new i(l);c.binaryType="arraybuffer",this.socket=s(c,t(this))}}else if(this.transportLibrary==="workersocket")this.socket=s(new e(l),t(this));else throw"Unknown transportLibrary: "+this.transportLibrary.toString()},a.prototype.close=function(){this.socket&&this.socket.close()},a.prototype.authenticate=function(l,c,u,h,d,f,g){var x={op:"auth",mac:l,client:c,dest:u,rand:h,t:d,level:f,end:g};this.callOnConnection(x)},a.prototype.sendEncodedMessage=function(l){var c=null,u=this;this.transportLibrary==="socket.io"?c=function(h){u.socket.emit("operation",h)}:c=function(h){u.socket.send(h)},this.isConnected?c(l):u.once("connection",function(){c(l)})},a.prototype.callOnConnection=function(l){this.transportOptions.encoder?this.transportOptions.encoder(l,this._sendFunc):this._sendFunc(JSON.stringify(l))},a.prototype.setStatusLevel=function(l,c){var u={op:"set_level",level:l,id:c};this.callOnConnection(u)},a.prototype.getActionServers=function(l,c){var u=new n({ros:this,name:"/rosapi/action_servers",serviceType:"rosapi/GetActionServers"}),h=new r({});typeof c=="function"?u.callService(h,function(d){l(d.action_servers)},function(d){c(d)}):u.callService(h,function(d){l(d.action_servers)})},a.prototype.getTopics=function(l,c){var u=new n({ros:this,name:"/rosapi/topics",serviceType:"rosapi/Topics"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d)},function(d){c(d)}):u.callService(h,function(d){l(d)})},a.prototype.getTopicsForType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/topics_for_type",serviceType:"rosapi/TopicsForType"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f.topics)},function(f){u(f)}):h.callService(d,function(f){c(f.topics)})},a.prototype.getServices=function(l,c){var u=new n({ros:this,name:"/rosapi/services",serviceType:"rosapi/Services"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d.services)},function(d){c(d)}):u.callService(h,function(d){l(d.services)})},a.prototype.getServicesForType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/services_for_type",serviceType:"rosapi/ServicesForType"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f.services)},function(f){u(f)}):h.callService(d,function(f){c(f.services)})},a.prototype.getServiceRequestDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/service_request_details",serviceType:"rosapi/ServiceRequestDetails"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f)},function(f){u(f)}):h.callService(d,function(f){c(f)})},a.prototype.getServiceResponseDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/service_response_details",serviceType:"rosapi/ServiceResponseDetails"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f)},function(f){u(f)}):h.callService(d,function(f){c(f)})},a.prototype.getNodes=function(l,c){var u=new n({ros:this,name:"/rosapi/nodes",serviceType:"rosapi/Nodes"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d.nodes)},function(d){c(d)}):u.callService(h,function(d){l(d.nodes)})},a.prototype.getNodeDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/node_details",serviceType:"rosapi/NodeDetails"}),d=new r({node:l});typeof u=="function"?h.callService(d,function(f){c(f.subscribing,f.publishing,f.services)},function(f){u(f)}):h.callService(d,function(f){c(f)})},a.prototype.getParams=function(l,c){var u=new n({ros:this,name:"/rosapi/get_param_names",serviceType:"rosapi/GetParamNames"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d.names)},function(d){c(d)}):u.callService(h,function(d){l(d.names)})},a.prototype.getTopicType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/topic_type",serviceType:"rosapi/TopicType"}),d=new r({topic:l});typeof u=="function"?h.callService(d,function(f){c(f.type)},function(f){u(f)}):h.callService(d,function(f){c(f.type)})},a.prototype.getServiceType=function(l,c,u){var h=new n({ros:this,name:"/rosapi/service_type",serviceType:"rosapi/ServiceType"}),d=new r({service:l});typeof u=="function"?h.callService(d,function(f){c(f.type)},function(f){u(f)}):h.callService(d,function(f){c(f.type)})},a.prototype.getMessageDetails=function(l,c,u){var h=new n({ros:this,name:"/rosapi/message_details",serviceType:"rosapi/MessageDetails"}),d=new r({type:l});typeof u=="function"?h.callService(d,function(f){c(f.typedefs)},function(f){u(f)}):h.callService(d,function(f){c(f.typedefs)})},a.prototype.decodeTypeDefs=function(l){var c=this,u=function(h,d){for(var f={},g=0;g<h.fieldnames.length;g++){var x=h.fieldarraylen[g],m=h.fieldnames[g],p=h.fieldtypes[g];if(p.indexOf("/")===-1)x===-1?f[m]=p:f[m]=[p];else{for(var w=!1,S=0;S<d.length;S++)if(d[S].type.toString()===p.toString()){w=d[S];break}if(w){var A=u(w,d);x===-1?f[m]=A:f[m]=[A]}else c.emit("error","Cannot find "+p+" in decodeTypeDefs")}}return f};return u(l[0],l)},a.prototype.getTopicsAndRawTypes=function(l,c){var u=new n({ros:this,name:"/rosapi/topics_and_raw_types",serviceType:"rosapi/TopicsAndRawTypes"}),h=new r;typeof c=="function"?u.callService(h,function(d){l(d)},function(d){c(d)}):u.callService(h,function(d){l(d)})},jr=a,jr}var $r,Xa;function pi(){if(Xa)return $r;Xa=1;var i=In;function e(t){i(this,t)}return $r=e,$r}var Kr,qa;function Ti(){if(qa)return Kr;qa=1;var i=Nn().EventEmitter2,e=pi();function t(n){n=n||{},this.ros=n.ros,this.name=n.name,this.messageType=n.messageType,this.isAdvertised=!1,this.compression=n.compression||"none",this.throttle_rate=n.throttle_rate||0,this.latch=n.latch||!1,this.queue_size=n.queue_size||100,this.queue_length=n.queue_length||0,this.reconnect_on_close=n.reconnect_on_close!==void 0?n.reconnect_on_close:!0,this.compression&&this.compression!=="png"&&this.compression!=="cbor"&&this.compression!=="cbor-raw"&&this.compression!=="none"&&(this.emit("warning",this.compression+" compression is not supported. No compression will be used."),this.compression="none"),this.throttle_rate<0&&(this.emit("warning",this.throttle_rate+" is not allowed. Set to 0"),this.throttle_rate=0);var r=this;this.reconnect_on_close?this.callForSubscribeAndAdvertise=function(s){r.ros.callOnConnection(s),r.waitForReconnect=!1,r.reconnectFunc=function(){r.waitForReconnect||(r.waitForReconnect=!0,r.ros.callOnConnection(s),r.ros.once("connection",function(){r.waitForReconnect=!1}))},r.ros.on("close",r.reconnectFunc)}:this.callForSubscribeAndAdvertise=this.ros.callOnConnection,this._messageCallback=function(s){r.emit("message",new e(s))}}return t.prototype.__proto__=i.prototype,t.prototype.subscribe=function(n){typeof n=="function"&&this.on("message",n),!this.subscribeId&&(this.ros.on(this.name,this._messageCallback),this.subscribeId="subscribe:"+this.name+":"+ ++this.ros.idCounter,this.callForSubscribeAndAdvertise({op:"subscribe",id:this.subscribeId,type:this.messageType,topic:this.name,compression:this.compression,throttle_rate:this.throttle_rate,queue_length:this.queue_length}))},t.prototype.unsubscribe=function(n){n&&(this.off("message",n),this.listeners("message").length)||this.subscribeId&&(this.ros.off(this.name,this._messageCallback),this.reconnect_on_close&&this.ros.off("close",this.reconnectFunc),this.emit("unsubscribe"),this.ros.callOnConnection({op:"unsubscribe",id:this.subscribeId,topic:this.name}),this.subscribeId=null)},t.prototype.advertise=function(){if(!this.isAdvertised&&(this.advertiseId="advertise:"+this.name+":"+ ++this.ros.idCounter,this.callForSubscribeAndAdvertise({op:"advertise",id:this.advertiseId,type:this.messageType,topic:this.name,latch:this.latch,queue_size:this.queue_size}),this.isAdvertised=!0,!this.reconnect_on_close)){var n=this;this.ros.on("close",function(){n.isAdvertised=!1})}},t.prototype.unadvertise=function(){this.isAdvertised&&(this.reconnect_on_close&&this.ros.off("close",this.reconnectFunc),this.emit("unadvertise"),this.ros.callOnConnection({op:"unadvertise",id:this.advertiseId,topic:this.name}),this.isAdvertised=!1)},t.prototype.publish=function(n){this.isAdvertised||this.advertise(),this.ros.idCounter++;var r={op:"publish",id:"publish:"+this.name+":"+this.ros.idCounter,topic:this.name,msg:n,latch:this.latch};this.ros.callOnConnection(r)},Kr=t,Kr}var Zr,Ya;function Cu(){if(Ya)return Zr;Ya=1;var i=Sr(),e=wi();function t(n){n=n||{},this.ros=n.ros,this.name=n.name}return t.prototype.get=function(n,r){var s=new i({ros:this.ros,name:"/rosapi/get_param",serviceType:"rosapi/GetParam"}),o=new e({name:this.name});s.callService(o,function(a){var l=JSON.parse(a.value);n(l)},r)},t.prototype.set=function(n,r,s){var o=new i({ros:this.ros,name:"/rosapi/set_param",serviceType:"rosapi/SetParam"}),a=new e({name:this.name,value:JSON.stringify(n)});o.callService(a,r,s)},t.prototype.delete=function(n,r){var s=new i({ros:this.ros,name:"/rosapi/delete_param",serviceType:"rosapi/DeleteParam"}),o=new e({name:this.name});s.callService(o,n,r)},Zr=t,Zr}var ja;function Ru(){if(ja)return Ur.exports;ja=1;var i=pa(),e=Ur.exports={Ros:ma(),Topic:Ti(),Message:pi(),Param:Cu(),Service:Sr(),ServiceRequest:wi(),ServiceResponse:lc()};return i(e.Ros,["Param","Service","Topic"],e),Ur.exports}var Jr={exports:{}},Qr,$a;function cc(){if($a)return Qr;$a=1;var i=Ti(),e=pi(),t=Nn().EventEmitter2;function n(r){var s=this;r=r||{},this.ros=r.ros,this.serverName=r.serverName,this.actionName=r.actionName,this.timeout=r.timeout,this.omitFeedback=r.omitFeedback,this.omitStatus=r.omitStatus,this.omitResult=r.omitResult,this.goals={};var o=!1;this.feedbackListener=new i({ros:this.ros,name:this.serverName+"/feedback",messageType:this.actionName+"Feedback"}),this.statusListener=new i({ros:this.ros,name:this.serverName+"/status",messageType:"actionlib_msgs/GoalStatusArray"}),this.resultListener=new i({ros:this.ros,name:this.serverName+"/result",messageType:this.actionName+"Result"}),this.goalTopic=new i({ros:this.ros,name:this.serverName+"/goal",messageType:this.actionName+"Goal"}),this.cancelTopic=new i({ros:this.ros,name:this.serverName+"/cancel",messageType:"actionlib_msgs/GoalID"}),this.goalTopic.advertise(),this.cancelTopic.advertise(),this.omitStatus||this.statusListener.subscribe(function(a){o=!0,a.status_list.forEach(function(l){var c=s.goals[l.goal_id.id];c&&c.emit("status",l)})}),this.omitFeedback||this.feedbackListener.subscribe(function(a){var l=s.goals[a.status.goal_id.id];l&&(l.emit("status",a.status),l.emit("feedback",a.feedback))}),this.omitResult||this.resultListener.subscribe(function(a){var l=s.goals[a.status.goal_id.id];l&&(l.emit("status",a.status),l.emit("result",a.result))}),this.timeout&&setTimeout(function(){o||s.emit("timeout")},this.timeout)}return n.prototype.__proto__=t.prototype,n.prototype.cancel=function(){var r=new e;this.cancelTopic.publish(r)},n.prototype.dispose=function(){this.goalTopic.unadvertise(),this.cancelTopic.unadvertise(),this.omitStatus||this.statusListener.unsubscribe(),this.omitFeedback||this.feedbackListener.unsubscribe(),this.omitResult||this.resultListener.unsubscribe()},Qr=n,Qr}var es,Ka;function Pu(){if(Ka)return es;Ka=1;var i=Ti();pi();var e=Nn().EventEmitter2;function t(n){var r=this;n=n||{},this.ros=n.ros,this.serverName=n.serverName,this.actionName=n.actionName;var s=new i({ros:this.ros,name:this.serverName+"/goal",messageType:this.actionName+"Goal"}),o=new i({ros:this.ros,name:this.serverName+"/feedback",messageType:this.actionName+"Feedback"}),a=new i({ros:this.ros,name:this.serverName+"/status",messageType:"actionlib_msgs/GoalStatusArray"}),l=new i({ros:this.ros,name:this.serverName+"/result",messageType:this.actionName+"Result"});s.subscribe(function(c){r.emit("goal",c)}),a.subscribe(function(c){c.status_list.forEach(function(u){r.emit("status",u)})}),o.subscribe(function(c){r.emit("status",c.status),r.emit("feedback",c.feedback)}),l.subscribe(function(c){r.emit("status",c.status),r.emit("result",c.result)})}return t.prototype.__proto__=e.prototype,es=t,es}var ts,Za;function uc(){if(Za)return ts;Za=1;var i=pi(),e=Nn().EventEmitter2;function t(n){var r=this;this.actionClient=n.actionClient,this.goalMessage=n.goalMessage,this.isFinished=!1;var s=new Date;this.goalID="goal_"+Math.random()+"_"+s.getTime(),this.goalMessage=new i({goal_id:{stamp:{secs:0,nsecs:0},id:this.goalID},goal:this.goalMessage}),this.on("status",function(o){r.status=o}),this.on("result",function(o){r.isFinished=!0,r.result=o}),this.on("feedback",function(o){r.feedback=o}),this.actionClient.goals[this.goalID]=this}return t.prototype.__proto__=e.prototype,t.prototype.send=function(n){var r=this;r.actionClient.goalTopic.publish(r.goalMessage),n&&setTimeout(function(){r.isFinished||r.emit("timeout")},n)},t.prototype.cancel=function(){var n=new i({id:this.goalID});this.actionClient.cancelTopic.publish(n)},ts=t,ts}var ns,Ja;function Lu(){if(Ja)return ns;Ja=1;var i=Ti(),e=pi(),t=Nn().EventEmitter2;function n(r){var s=this;r=r||{},this.ros=r.ros,this.serverName=r.serverName,this.actionName=r.actionName,this.feedbackPublisher=new i({ros:this.ros,name:this.serverName+"/feedback",messageType:this.actionName+"Feedback"}),this.feedbackPublisher.advertise();var o=new i({ros:this.ros,name:this.serverName+"/status",messageType:"actionlib_msgs/GoalStatusArray"});o.advertise(),this.resultPublisher=new i({ros:this.ros,name:this.serverName+"/result",messageType:this.actionName+"Result"}),this.resultPublisher.advertise();var a=new i({ros:this.ros,name:this.serverName+"/goal",messageType:this.actionName+"Goal"}),l=new i({ros:this.ros,name:this.serverName+"/cancel",messageType:"actionlib_msgs/GoalID"});this.statusMessage=new e({header:{stamp:{secs:0,nsecs:100},frame_id:""},status_list:[]}),this.currentGoal=null,this.nextGoal=null,a.subscribe(function(u){s.currentGoal?(s.nextGoal=u,s.emit("cancel")):(s.statusMessage.status_list=[{goal_id:u.goal_id,status:1}],s.currentGoal=u,s.emit("goal",u.goal))});var c=function(u,h){return u.secs>h.secs?!1:u.secs<h.secs?!0:u.nsecs<h.nsecs};l.subscribe(function(u){u.stamp.secs===0&&u.stamp.secs===0&&u.id===""?(s.nextGoal=null,s.currentGoal&&s.emit("cancel")):(s.currentGoal&&u.id===s.currentGoal.goal_id.id?s.emit("cancel"):s.nextGoal&&u.id===s.nextGoal.goal_id.id&&(s.nextGoal=null),s.nextGoal&&c(s.nextGoal.goal_id.stamp,u.stamp)&&(s.nextGoal=null),s.currentGoal&&c(s.currentGoal.goal_id.stamp,u.stamp)&&s.emit("cancel"))}),setInterval(function(){var u=new Date,h=Math.floor(u.getTime()/1e3),d=Math.round(1e9*(u.getTime()/1e3-h));s.statusMessage.header.stamp.secs=h,s.statusMessage.header.stamp.nsecs=d,o.publish(s.statusMessage)},500)}return n.prototype.__proto__=t.prototype,n.prototype.setSucceeded=function(r){var s=new e({status:{goal_id:this.currentGoal.goal_id,status:3},result:r});this.resultPublisher.publish(s),this.statusMessage.status_list=[],this.nextGoal?(this.currentGoal=this.nextGoal,this.nextGoal=null,this.emit("goal",this.currentGoal.goal)):this.currentGoal=null},n.prototype.setAborted=function(r){var s=new e({status:{goal_id:this.currentGoal.goal_id,status:4},result:r});this.resultPublisher.publish(s),this.statusMessage.status_list=[],this.nextGoal?(this.currentGoal=this.nextGoal,this.nextGoal=null,this.emit("goal",this.currentGoal.goal)):this.currentGoal=null},n.prototype.sendFeedback=function(r){var s=new e({status:{goal_id:this.currentGoal.goal_id,status:1},feedback:r});this.feedbackPublisher.publish(s)},n.prototype.setPreempted=function(){this.statusMessage.status_list=[];var r=new e({status:{goal_id:this.currentGoal.goal_id,status:2}});this.resultPublisher.publish(r),this.nextGoal?(this.currentGoal=this.nextGoal,this.nextGoal=null,this.emit("goal",this.currentGoal.goal)):this.currentGoal=null},ns=n,ns}var Qa;function Uu(){if(Qa)return Jr.exports;Qa=1;var i=ma(),e=pa(),t=Jr.exports={ActionClient:cc(),ActionListener:Pu(),Goal:uc(),SimpleActionServer:Lu()};return e(i,["ActionClient","SimpleActionServer"],t),Jr.exports}var is,eo;function On(){if(eo)return is;eo=1;function i(e){e=e||{},this.x=e.x||0,this.y=e.y||0,this.z=e.z||0}return i.prototype.add=function(e){this.x+=e.x,this.y+=e.y,this.z+=e.z},i.prototype.subtract=function(e){this.x-=e.x,this.y-=e.y,this.z-=e.z},i.prototype.multiplyQuaternion=function(e){var t=e.w*this.x+e.y*this.z-e.z*this.y,n=e.w*this.y+e.z*this.x-e.x*this.z,r=e.w*this.z+e.x*this.y-e.y*this.x,s=-e.x*this.x-e.y*this.y-e.z*this.z;this.x=t*e.w+s*-e.x+n*-e.z-r*-e.y,this.y=n*e.w+s*-e.y+r*-e.x-t*-e.z,this.z=r*e.w+s*-e.z+t*-e.y-n*-e.x},i.prototype.clone=function(){return new i(this)},is=i,is}var rs,to;function Ai(){if(to)return rs;to=1;function i(e){e=e||{},this.x=e.x||0,this.y=e.y||0,this.z=e.z||0,this.w=typeof e.w=="number"?e.w:1}return i.prototype.conjugate=function(){this.x*=-1,this.y*=-1,this.z*=-1},i.prototype.norm=function(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)},i.prototype.normalize=function(){var e=Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w);e===0?(this.x=0,this.y=0,this.z=0,this.w=1):(e=1/e,this.x=this.x*e,this.y=this.y*e,this.z=this.z*e,this.w=this.w*e)},i.prototype.invert=function(){this.conjugate(),this.normalize()},i.prototype.multiply=function(e){var t=this.x*e.w+this.y*e.z-this.z*e.y+this.w*e.x,n=-this.x*e.z+this.y*e.w+this.z*e.x+this.w*e.y,r=this.x*e.y-this.y*e.x+this.z*e.w+this.w*e.z,s=-this.x*e.x-this.y*e.y-this.z*e.z+this.w*e.w;this.x=t,this.y=n,this.z=r,this.w=s},i.prototype.clone=function(){return new i(this)},rs=i,rs}var ss,no;function ga(){if(no)return ss;no=1;var i=On(),e=Ai();function t(n){n=n||{},this.position=new i(n.position),this.orientation=new e(n.orientation)}return t.prototype.applyTransform=function(n){this.position.multiplyQuaternion(n.rotation),this.position.add(n.translation);var r=n.rotation.clone();r.multiply(this.orientation),this.orientation=r},t.prototype.clone=function(){return new t(this)},t.prototype.multiply=function(n){var r=n.clone();return r.applyTransform({rotation:this.orientation,translation:this.position}),r},t.prototype.getInverse=function(){var n=this.clone();return n.orientation.invert(),n.position.multiplyQuaternion(n.orientation),n.position.x*=-1,n.position.y*=-1,n.position.z*=-1,n},ss=t,ss}var as,ro;function hc(){if(ro)return as;ro=1;var i=On(),e=Ai();function t(n){n=n||{},this.translation=new i(n.translation),this.rotation=new e(n.rotation)}return t.prototype.clone=function(){return new t(this)},as=t,as}var os,so;function Du(){return so||(so=1,os={Pose:ga(),Quaternion:Ai(),Transform:hc(),Vector3:On()}),os}var ls={exports:{}},cs,ao;function Iu(){if(ao)return cs;ao=1;var i=cc(),e=uc(),t=Sr(),n=wi(),r=Ti(),s=hc();function o(a){a=a||{},this.ros=a.ros,this.fixedFrame=a.fixedFrame||"base_link",this.angularThres=a.angularThres||2,this.transThres=a.transThres||.01,this.rate=a.rate||10,this.updateDelay=a.updateDelay||50;var l=a.topicTimeout||2,c=Math.floor(l),u=Math.floor((l-c)*1e9);this.topicTimeout={secs:c,nsecs:u},this.serverName=a.serverName||"/tf2_web_republisher",this.repubServiceName=a.repubServiceName||"/republish_tfs",this.currentGoal=!1,this.currentTopic=!1,this.frameInfos={},this.republisherUpdateRequested=!1,this._subscribeCB=null,this._isDisposed=!1,this.actionClient=new i({ros:a.ros,serverName:this.serverName,actionName:"tf2_web_republisher/TFSubscriptionAction",omitStatus:!0,omitResult:!0}),this.serviceClient=new t({ros:a.ros,name:this.repubServiceName,serviceType:"tf2_web_republisher/RepublishTFs"})}return o.prototype.processTFArray=function(a){a.transforms.forEach(function(l){var c=l.child_frame_id;c[0]==="/"&&(c=c.substring(1));var u=this.frameInfos[c];u&&(u.transform=new s({translation:l.transform.translation,rotation:l.transform.rotation}),u.cbs.forEach(function(h){h(u.transform)}))},this)},o.prototype.updateGoal=function(){var a={source_frames:Object.keys(this.frameInfos),target_frame:this.fixedFrame,angular_thres:this.angularThres,trans_thres:this.transThres,rate:this.rate};if(this.ros.groovyCompatibility)this.currentGoal&&this.currentGoal.cancel(),this.currentGoal=new e({actionClient:this.actionClient,goalMessage:a}),this.currentGoal.on("feedback",this.processTFArray.bind(this)),this.currentGoal.send();else{a.timeout=this.topicTimeout;var l=new n(a);this.serviceClient.callService(l,this.processResponse.bind(this))}this.republisherUpdateRequested=!1},o.prototype.processResponse=function(a){this._isDisposed||(this.currentTopic&&this.currentTopic.unsubscribe(this._subscribeCB),this.currentTopic=new r({ros:this.ros,name:a.topic_name,messageType:"tf2_web_republisher/TFArray"}),this._subscribeCB=this.processTFArray.bind(this),this.currentTopic.subscribe(this._subscribeCB))},o.prototype.subscribe=function(a,l){a[0]==="/"&&(a=a.substring(1)),this.frameInfos[a]?this.frameInfos[a].transform&&l(this.frameInfos[a].transform):(this.frameInfos[a]={cbs:[]},this.republisherUpdateRequested||(setTimeout(this.updateGoal.bind(this),this.updateDelay),this.republisherUpdateRequested=!0)),this.frameInfos[a].cbs.push(l)},o.prototype.unsubscribe=function(a,l){a[0]==="/"&&(a=a.substring(1));for(var c=this.frameInfos[a],u=c&&c.cbs||[],h=u.length;h--;)u[h]===l&&u.splice(h,1);(!l||u.length===0)&&delete this.frameInfos[a]},o.prototype.dispose=function(){this._isDisposed=!0,this.actionClient.dispose(),this.currentTopic&&this.currentTopic.unsubscribe(this._subscribeCB)},cs=o,cs}var oo;function Nu(){if(oo)return ls.exports;oo=1;var i=ma(),e=pa(),t=ls.exports={TFClient:Iu()};return e(i,["TFClient"],t),ls.exports}var us,lo;function Ci(){return lo||(lo=1,us={URDF_SPHERE:0,URDF_BOX:1,URDF_CYLINDER:2,URDF_MESH:3}),us}var hs,co;function dc(){if(co)return hs;co=1;var i=On(),e=Ci();function t(n){this.dimension=null,this.type=e.URDF_BOX;var r=n.xml.getAttribute("size").split(" ");this.dimension=new i({x:parseFloat(r[0]),y:parseFloat(r[1]),z:parseFloat(r[2])})}return hs=t,hs}var ds,uo;function fc(){if(uo)return ds;uo=1;function i(e){var t=e.xml.getAttribute("rgba").split(" ");this.r=parseFloat(t[0]),this.g=parseFloat(t[1]),this.b=parseFloat(t[2]),this.a=parseFloat(t[3])}return ds=i,ds}var fs,ho;function pc(){if(ho)return fs;ho=1;var i=Ci();function e(t){this.type=i.URDF_CYLINDER,this.length=parseFloat(t.xml.getAttribute("length")),this.radius=parseFloat(t.xml.getAttribute("radius"))}return fs=e,fs}var ps,fo;function _a(){if(fo)return ps;fo=1;var i=fc();function e(n){this.textureFilename=null,this.color=null,this.name=n.xml.getAttribute("name");var r=n.xml.getElementsByTagName("texture");r.length>0&&(this.textureFilename=r[0].getAttribute("filename"));var s=n.xml.getElementsByTagName("color");s.length>0&&(this.color=new i({xml:s[0]}))}e.prototype.isLink=function(){return this.color===null&&this.textureFilename===null};var t=In;return e.prototype.assign=function(n){return t(this,n)},ps=e,ps}var ms,po;function mc(){if(po)return ms;po=1;var i=On(),e=Ci();function t(n){this.scale=null,this.type=e.URDF_MESH,this.filename=n.xml.getAttribute("filename");var r=n.xml.getAttribute("scale");if(r){var s=r.split(" ");this.scale=new i({x:parseFloat(s[0]),y:parseFloat(s[1]),z:parseFloat(s[2])})}}return ms=t,ms}var gs,mo;function gc(){if(mo)return gs;mo=1;var i=Ci();function e(t){this.type=i.URDF_SPHERE,this.radius=parseFloat(t.xml.getAttribute("radius"))}return gs=e,gs}var _s,go;function _c(){if(go)return _s;go=1;var i=ga(),e=On(),t=Ai(),n=pc(),r=dc(),s=_a(),o=mc(),a=gc();function l(c){var u=c.xml;this.origin=null,this.geometry=null,this.material=null,this.name=c.xml.getAttribute("name");var h=u.getElementsByTagName("origin");if(h.length===0)this.origin=new i;else{var d=h[0].getAttribute("xyz"),f=new e;d&&(d=d.split(" "),f=new e({x:parseFloat(d[0]),y:parseFloat(d[1]),z:parseFloat(d[2])}));var g=h[0].getAttribute("rpy"),x=new t;if(g){g=g.split(" ");var m=parseFloat(g[0]),p=parseFloat(g[1]),w=parseFloat(g[2]),S=m/2,A=p/2,k=w/2,I=Math.sin(S)*Math.cos(A)*Math.cos(k)-Math.cos(S)*Math.sin(A)*Math.sin(k),P=Math.cos(S)*Math.sin(A)*Math.cos(k)+Math.sin(S)*Math.cos(A)*Math.sin(k),Y=Math.cos(S)*Math.cos(A)*Math.sin(k)-Math.sin(S)*Math.sin(A)*Math.cos(k),E=Math.cos(S)*Math.cos(A)*Math.cos(k)+Math.sin(S)*Math.sin(A)*Math.sin(k);x=new t({x:I,y:P,z:Y,w:E}),x.normalize()}this.origin=new i({position:f,orientation:x})}var b=u.getElementsByTagName("geometry");if(b.length>0){for(var C=b[0],H=null,O=0;O<C.childNodes.length;O++){var $=C.childNodes[O];if($.nodeType===1){H=$;break}}var z=H.nodeName;z==="sphere"?this.geometry=new a({xml:H}):z==="box"?this.geometry=new r({xml:H}):z==="cylinder"?this.geometry=new n({xml:H}):z==="mesh"?this.geometry=new o({xml:H}):console.warn("Unknown geometry type "+z)}var q=u.getElementsByTagName("material");q.length>0&&(this.material=new s({xml:q[0]}))}return _s=l,_s}var vs,_o;function vc(){if(_o)return vs;_o=1;var i=_c();function e(t){this.name=t.xml.getAttribute("name"),this.visuals=[];for(var n=t.xml.getElementsByTagName("visual"),r=0;r<n.length;r++)this.visuals.push(new i({xml:n[r]}))}return vs=e,vs}var xs,vo;function Ou(){if(vo)return xs;vo=1;var i=ga(),e=On(),t=Ai();function n(r){this.name=r.xml.getAttribute("name"),this.type=r.xml.getAttribute("type");var s=r.xml.getElementsByTagName("parent");s.length>0&&(this.parent=s[0].getAttribute("link"));var o=r.xml.getElementsByTagName("child");o.length>0&&(this.child=o[0].getAttribute("link"));var a=r.xml.getElementsByTagName("limit");a.length>0&&(this.minval=parseFloat(a[0].getAttribute("lower")),this.maxval=parseFloat(a[0].getAttribute("upper")));var l=r.xml.getElementsByTagName("origin");if(l.length===0)this.origin=new i;else{var c=l[0].getAttribute("xyz"),u=new e;c&&(c=c.split(" "),u=new e({x:parseFloat(c[0]),y:parseFloat(c[1]),z:parseFloat(c[2])}));var h=l[0].getAttribute("rpy"),d=new t;if(h){h=h.split(" ");var f=parseFloat(h[0]),g=parseFloat(h[1]),x=parseFloat(h[2]),m=f/2,p=g/2,w=x/2,S=Math.sin(m)*Math.cos(p)*Math.cos(w)-Math.cos(m)*Math.sin(p)*Math.sin(w),A=Math.cos(m)*Math.sin(p)*Math.cos(w)+Math.sin(m)*Math.cos(p)*Math.sin(w),k=Math.cos(m)*Math.cos(p)*Math.sin(w)-Math.sin(m)*Math.sin(p)*Math.cos(w),I=Math.cos(m)*Math.cos(p)*Math.cos(w)+Math.sin(m)*Math.sin(p)*Math.sin(w);d=new t({x:S,y:A,z:k,w:I}),d.normalize()}this.origin=new i({position:u,orientation:d})}}return xs=n,xs}var _i={},xo;function Fu(){return xo||(xo=1,_i.DOMImplementation=window.DOMImplementation,_i.XMLSerializer=window.XMLSerializer,_i.DOMParser=window.DOMParser),_i}var ys,yo;function Bu(){if(yo)return ys;yo=1;var i=_a(),e=vc(),t=Ou(),n=Fu().DOMParser;function r(s){s=s||{};var o=s.xml,a=s.string;if(this.materials={},this.links={},this.joints={},a){var l=new n;o=l.parseFromString(a,"text/xml")}var c=o.documentElement;this.name=c.getAttribute("name");for(var u=c.childNodes,h=0;h<u.length;h++){var d=u[h];if(d.tagName==="material"){var f=new i({xml:d});this.materials[f.name]!==void 0?this.materials[f.name].isLink()?this.materials[f.name].assign(f):console.warn("Material "+f.name+"is not unique."):this.materials[f.name]=f}else if(d.tagName==="link"){var g=new e({xml:d});if(this.links[g.name]!==void 0)console.warn("Link "+g.name+" is not unique.");else{for(var x=0;x<g.visuals.length;x++){var m=g.visuals[x].material;m!==null&&m.name&&(this.materials[m.name]!==void 0?g.visuals[x].material=this.materials[m.name]:this.materials[m.name]=m)}this.links[g.name]=g}}else if(d.tagName==="joint"){var p=new t({xml:d});this.joints[p.name]=p}}}return ys=r,ys}var Ss,So;function ku(){return So||(So=1,Ss=In({UrdfBox:dc(),UrdfColor:fc(),UrdfCylinder:pc(),UrdfLink:vc(),UrdfMaterial:_a(),UrdfMesh:mc(),UrdfModel:Bu(),UrdfSphere:gc(),UrdfVisual:_c()},Ci())),Ss}var mi=ac.ROSLIB||{REVISION:"1.4.1"},Ri=In;Ri(mi,Ru());Ri(mi,Uu());Ri(mi,Du());Ri(mi,Nu());Ri(mi,ku());var zu=mi;const Ze=pu(zu);function Vu(i,e){return i.id===17||i.id===16?{data:0}:i.id===5?{data:2}:i.id===18?{data:3}:i.id===19?{data:Number(e.globalZ)||0}:i.id===23?{data:Number(e.speed)||0}:i.id===24?{data:Number(e.fixedSpeed)||0}:{data:1}}function Gu(i,e){return i.id===2?{position:{x:Number(e.startX)||0,y:Number(e.startY)||0,z:Number(e.startZ)||0},orientation:{x:Number(e.globalX)||0,y:Number(e.globalY)||0,z:Number(e.stepX)||0,w:Number(e.stepY)||0}}:i.id===6?{position:{x:Number(e.fixedX)||0,y:Number(e.fixedY)||0,z:Number(e.fixedZ)||0},orientation:{x:Number(e.fixedTheta)||0,y:0,z:0,w:1}}:i.id===7?{position:{x:Number(e.startX)||0,y:Number(e.startY)||0,z:Number(e.startZ)||0},orientation:{x:Number(e.speed)||0,y:0,z:0,w:1}}:i.id===22?{position:{x:Number(e.fixedX)||0,y:Number(e.fixedY)||0,z:Number(e.fixedZ)||0},orientation:{x:0,y:0,z:0,w:1}}:{position:{x:0,y:0,z:0},orientation:{x:0,y:0,z:0,w:1}}}class Hu{constructor({rosConnection:e,callbacks:t={}}){this.rosConnection=e,this.callbacks=t,this.publisherCache=new Map,this.booleanStates=new Map}reset(){this.publisherCache.clear()}handle(e,t){var a,l,c,u,h,d,f,g,x,m,p,w;const n=sc.find(S=>S.id===e);if(!n){(l=(a=this.callbacks).onResultMessage)==null||l.call(a,`未识别的老前端命令: ${e}`),(u=(c=this.callbacks).onLog)==null||u.call(c,`未识别的老前端命令: ${e}`,"warn");return}const r=this.rosConnection.getResources();if(!(r!=null&&r.ros)){(d=(h=this.callbacks).onResultMessage)==null||d.call(h,`ROS 未连接，无法执行: ${n.name}`),(g=(f=this.callbacks).onLog)==null||g.call(f,`ROS 未连接，无法执行: ${n.name}`,"warn");return}const s=this.getOrCreatePublisher(r.ros,n.topic,n.type),o=this.buildMessagePayload(n,t);s.publish(new Ze.Message(o)),(m=(x=this.callbacks).onResultMessage)==null||m.call(x,`已发送 ${n.name} -> ${n.topic}`),(w=(p=this.callbacks).onLog)==null||w.call(p,`已发送 ${n.name} -> ${n.topic}`,"success")}getOrCreatePublisher(e,t,n){const r=`${t}|${n}`;if(this.publisherCache.has(r))return this.publisherCache.get(r);const s=new Ze.Topic({ros:e,name:t,messageType:n});return s.advertise(),this.publisherCache.set(r,s),s}buildMessagePayload(e,t){if(e.type==="std_msgs/Bool"){const n=!this.booleanStates.get(e.id);return this.booleanStates.set(e.id,n),{data:n}}return e.type==="std_msgs/Float32"?Vu(e,t):e.type==="geometry_msgs/Pose"?Gu(e,t):{}}}function Wu({pathname:i="/",savedPreferences:e=null,currentHost:t="localhost"}={}){if(i&&i.length>1)try{const n=decodeURIComponent(i.substring(1));if(n.startsWith("ws://")||n.startsWith("wss://"))return n}catch{}return t?`ws://${t}:9090`:e!=null&&e.ip?`ws://${e.ip}:${e.port||"9090"}`:"ws://localhost:9090"}const Xu="/Scepter/ir/image_raw",qu="/web/pointAI/set_workspace_quad",Yu="/web/pointAI/run_workspace_s2",ju="/pointAI/manual_workspace_quad_pixels",$u="/pointAI/manual_workspace_s2_result_raw",Ku="/pointAI/result_image_raw",Zu="/Scepter/worldCoord/world_coord",Ju="/Scepter/worldCoord/raw_world_coord",Qu="/coordinate_point",eh="/cabin/pseudo_slam_markers",th="/tf",nh="/tf_static",ih="/web/cabin/start_pseudo_slam_scan",rh="/web/cabin/start_global_work",sh="/web/cabin/run_bind_path_direct_test",ah="/cabin/set_execution_mode",oh="tie_robot_msgs/StartPseudoSlamScanTaskAction",lh="tie_robot_msgs/StartGlobalWorkTaskAction",ch="tie_robot_msgs/RunBindPathDirectTestTaskAction",uh="tie_robot_msgs/SetExecutionMode";class hh{constructor(e={}){this.callbacks=e,this.ros=null,this.resources=null,this.topicSubscribers=[]}connect(){var t,n,r,s;const e=Wu({pathname:window.location.pathname,currentHost:window.location.hostname});(n=(t=this.callbacks).onConnectionInfo)==null||n.call(t,e,`正在连接 ${e} ...`,"info"),(s=(r=this.callbacks).onLog)==null||s.call(r,`开始连接 ROSBridge: ${e}`,"info"),this.ros=new Ze.Ros({url:e}),this.resources=this.buildResources(this.ros),this.ros.on("connection",()=>{var o,a,l,c,u,h;this.resources.workspaceQuadPublisher.advertise(),this.resources.runWorkspaceS2Publisher.advertise(),this.bindSubscriptions(),(a=(o=this.callbacks).onConnectionInfo)==null||a.call(o,e,"ROS 连接成功，等待 IR 图像...","success"),(c=(l=this.callbacks).onRosReady)==null||c.call(l,this.resources),(h=(u=this.callbacks).onLog)==null||h.call(u,"ROS 连接成功，动作链与图像订阅已就绪","success")}),this.ros.on("error",o=>{var l,c,u,h,d,f;const a=(o==null?void 0:o.message)||String(o);(c=(l=this.callbacks).onConnectionInfo)==null||c.call(l,e,`ROS 连接失败: ${a}`,"error"),(h=(u=this.callbacks).onRosUnavailable)==null||h.call(u),(f=(d=this.callbacks).onLog)==null||f.call(d,`ROS 连接失败: ${a}`,"error")}),this.ros.on("close",()=>{var o,a,l,c,u,h;this.unbindSubscriptions(),(a=(o=this.callbacks).onConnectionInfo)==null||a.call(o,e,"ROS 连接已断开","warn"),(c=(l=this.callbacks).onRosUnavailable)==null||c.call(l),(h=(u=this.callbacks).onLog)==null||h.call(u,"ROS 连接已断开","warn")})}buildResources(e){return{ros:e,workspaceQuadPublisher:new Ze.Topic({ros:e,name:qu,messageType:"std_msgs/Float32MultiArray"}),runWorkspaceS2Publisher:new Ze.Topic({ros:e,name:Yu,messageType:"std_msgs/Bool"}),startPseudoSlamScanActionClient:new Ze.ActionClient({ros:e,serverName:ih,actionName:oh}),executionModeService:new Ze.Service({ros:e,name:ah,serviceType:uh}),startGlobalWorkActionClient:new Ze.ActionClient({ros:e,serverName:rh,actionName:lh}),runDirectBindPathTestActionClient:new Ze.ActionClient({ros:e,serverName:sh,actionName:ch})}}bindSubscriptions(){this.unbindSubscriptions();const e=[new Ze.Topic({ros:this.ros,name:Xu,messageType:"sensor_msgs/Image"}),new Ze.Topic({ros:this.ros,name:ju,messageType:"std_msgs/Float32MultiArray"}),new Ze.Topic({ros:this.ros,name:$u,messageType:"sensor_msgs/Image"}),new Ze.Topic({ros:this.ros,name:Ku,messageType:"sensor_msgs/Image"}),new Ze.Topic({ros:this.ros,name:Zu,messageType:"sensor_msgs/Image"}),new Ze.Topic({ros:this.ros,name:Ju,messageType:"sensor_msgs/Image"}),new Ze.Topic({ros:this.ros,name:Qu,messageType:"tie_robot_msgs/PointsArray"}),new Ze.Topic({ros:this.ros,name:eh,messageType:"visualization_msgs/MarkerArray"}),new Ze.Topic({ros:this.ros,name:th,messageType:"tf2_msgs/TFMessage"}),new Ze.Topic({ros:this.ros,name:nh,messageType:"tf2_msgs/TFMessage"})];e[0].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onBaseImage)==null?void 0:r.call(n,t)}),e[1].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onSavedWorkspacePayload)==null?void 0:r.call(n,Array.from(t.data||[]))}),e[2].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onS2Overlay)==null?void 0:r.call(n,t)}),e[3].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onExecutionOverlay)==null?void 0:r.call(n,t)}),e[4].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onPointCloudImage)==null?void 0:r.call(n,"filteredWorldCoord",t)}),e[5].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onPointCloudImage)==null?void 0:r.call(n,"rawWorldCoord",t)}),e[6].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onTiePoints)==null?void 0:r.call(n,t)}),e[7].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onPlanningMarkers)==null?void 0:r.call(n,t)}),e[8].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onTfMessage)==null?void 0:r.call(n,t)}),e[9].subscribe(t=>{var n,r;return(r=(n=this.callbacks).onTfMessage)==null?void 0:r.call(n,t)}),this.topicSubscribers=e}unbindSubscriptions(){this.topicSubscribers.forEach(e=>{try{e.unsubscribe()}catch{}}),this.topicSubscribers=[]}getResources(){return this.resources}isReady(){return!!(this.resources&&this.ros&&this.ros.isConnected)}}const xc=[{id:"ros",label:"ROS",kind:"connection"},{id:"chassis",label:"索驱",topic:"/robot/chassis_status",messageType:"std_msgs/Float32"},{id:"moduan",label:"末端",topic:"/robot/moduan_status",messageType:"std_msgs/Float32"},{id:"bindingGun",label:"绑扎枪",topic:"/robot/binding_gun_status",messageType:"std_msgs/Float32"}];function dh(i){return Number.isFinite(i)?i<0?"error":i>0?"success":"info":"warn"}class fh{constructor(e={}){this.callbacks=e,this.subscriptions=[],this.lastValues=new Map}setConnectionState(e,t){var n,r;(r=(n=this.callbacks).onStatusChip)==null||r.call(n,"ros",e,t)}start(e){this.stop(),xc.filter(t=>t.kind!=="connection").forEach(t=>{const n=new Ze.Topic({ros:e,name:t.topic,messageType:t.messageType});n.subscribe(r=>{var l,c,u,h;const s=Number(r==null?void 0:r.data),o=dh(s),a=`${t.label}: ${Number.isFinite(s)?s:"无效值"}`;(c=(l=this.callbacks).onStatusChip)==null||c.call(l,t.id,o,a),this.lastValues.get(t.id)!==s&&((h=(u=this.callbacks).onLog)==null||h.call(u,`状态变化 ${t.label} -> ${a}`,o),this.lastValues.set(t.id,s))}),this.subscriptions.push(n)})}stop(){this.subscriptions.forEach(e=>{try{e.unsubscribe()}catch{}}),this.subscriptions=[]}}function ph(i,e,t){return{x:Math.min(Math.max(Math.round(i.x),0),Math.max(e-1,0)),y:Math.min(Math.max(Math.round(i.y),0),Math.max(t-1,0))}}function Ms({clientX:i,clientY:e,rect:t,imageWidth:n,imageHeight:r}){const s=Math.max((t==null?void 0:t.width)||0,1),o=Math.max((t==null?void 0:t.height)||0,1),a=(i-((t==null?void 0:t.left)||0))/s*n,l=(e-((t==null?void 0:t.top)||0))/o*r;return ph({x:a,y:l},n,r)}function yc(i){if(!Array.isArray(i)||i.length!==4)throw new Error("Workspace quad requires exactly four points");return i.flatMap(e=>[Math.round(e.x),Math.round(e.y)])}function mh(i){if(!Array.isArray(i)||i.length!==8)throw new Error("Workspace quad payload requires exactly eight values");const e=[];for(let t=0;t<i.length;t+=2)e.push({x:Number(i[t]),y:Number(i[t+1])});return e}function gh(i,e,t=12){if(!Array.isArray(i)||!e)return-1;let n=-1,r=Number.POSITIVE_INFINITY;const s=t*t;return i.forEach((o,a)=>{const l=Number(o.x)-Number(e.x),c=Number(o.y)-Number(e.y),u=l*l+c*c;u<=s&&u<r&&(r=u,n=a)}),n}function _h(i,e,t){return(i||[]).map((n,r)=>r!==e?n:{x:Number(t.x),y:Number(t.y)})}function Sc(i){if(Array.isArray(i))return Uint8ClampedArray.from(i);if(typeof i=="string"){const e=atob(i),t=new Uint8ClampedArray(e.length);for(let n=0;n<e.length;n+=1)t[n]=e.charCodeAt(n);return t}return i instanceof Uint8Array||i instanceof Uint8ClampedArray?new Uint8ClampedArray(i):(i==null?void 0:i.buffer)instanceof ArrayBuffer?new Uint8ClampedArray(i.buffer):null}function Mo(i,e){const t=new Uint32Array(256);for(let o=0;o<i.length;o+=1)t[i[o]]+=1;const n=Math.min(Math.max(e,0),100),r=Math.max(Math.ceil(n/100*i.length),1);let s=0;for(let o=0;o<t.length;o+=1)if(s+=t[o],s>=r)return o;return 255}function vh(i,{lowClipPercent:e=1,highClipPercent:t=99}={}){const n=Mo(i,e),r=Mo(i,t);if(r<=n)return Uint8ClampedArray.from(i);const s=new Uint8ClampedArray(i.length),o=255/(r-n);for(let a=0;a<i.length;a+=1){const l=Math.round((i[a]-n)*o);s[a]=Math.min(Math.max(l,0),255)}return s}function xh(i,e=1){const t=Number.isFinite(e)&&e>0?e:1,n=new Uint8ClampedArray(i.length);for(let r=0;r<i.length;r+=1)n[r]=Math.round(255*(i[r]/255)**t);return n}function yh(i){const e=new Uint32Array(256);for(let a=0;a<i.length;a+=1)e[i[a]]+=1;let t=0;const n=new Uint32Array(256);let r=0;for(let a=0;a<e.length;a+=1)r+=e[a],n[a]=r,t===0&&e[a]>0&&(t=r);if(t===i.length)return Uint8ClampedArray.from(i);const s=new Uint8ClampedArray(i.length),o=Math.max(i.length-t,1);for(let a=0;a<i.length;a+=1)s[a]=Math.round((n[i[a]]-t)/o*255);return s}function Sh(i,{mode:e="auto",gamma:t=.85}={}){if(e==="raw")return Uint8ClampedArray.from(i);let n=vh(i);return e==="strong"&&(n=yh(n)),xh(n,t)}function bo(i,e){const t=Sc(i.data),n=Number(i.width)||0,r=Number(i.height)||0,s=String(i.encoding||"").toLowerCase();if(!t||n<=0||r<=0)throw new Error("图像数据无效");const o=new Uint8ClampedArray(n*r*4);if(s.includes("mono8")||s.includes("8uc1")){const a=Sh(t,e);for(let l=0;l<n*r;l+=1){const c=a[l]??0,u=l*4;o[u]=c,o[u+1]=c,o[u+2]=c,o[u+3]=255}return new ImageData(o,n,r)}if(s.includes("rgb8")||s.includes("bgr8")){for(let a=0;a<n*r;a+=1){const l=a*3,c=a*4,u=s.includes("bgr8")?[t[l+2],t[l+1],t[l]]:[t[l],t[l+1],t[l+2]];o[c]=u[0]??0,o[c+1]=u[1]??0,o[c+2]=u[2]??0,o[c+3]=255}return new ImageData(o,n,r)}throw new Error(`暂不支持的图像编码: ${i.encoding}`)}function Mh(i,{sampleStep:e=4,maxPoints:t=24e3}={}){const n=Sc(i==null?void 0:i.data),r=Number(i==null?void 0:i.width)||0,s=Number(i==null?void 0:i.height)||0;if(!n||r<=0||s<=0)return{positions:new Float32Array,count:0};const o=new DataView(n.buffer,n.byteOffset,n.byteLength),a=Math.max(1,Number(e)||1),l=[];let c=0;for(let u=0;u<s;u+=a)for(let h=0;h<r;h+=a){const d=(u*r+h)*12;if(d+12>o.byteLength)continue;const f=o.getFloat32(d,!0),g=o.getFloat32(d+4,!0),x=o.getFloat32(d+8,!0);if(!(!Number.isFinite(f)||!Number.isFinite(g)||!Number.isFinite(x))&&!(f===0&&g===0&&x===0)&&(l.push(f/1e3,g/1e3,x/1e3),c+=1,c>=t))return{positions:Float32Array.from(l),count:c}}return{positions:Float32Array.from(l),count:c}}class bh{constructor({rosConnection:e,workspaceView:t,callbacks:n={}}){this.rosConnection=e,this.workspaceView=t,this.callbacks=n}handle(e){switch(e){case"submitQuad":return this.publishWorkspaceQuad();case"runSavedS2":return this.triggerSavedWorkspaceS2();case"scanPlan":return this.triggerPseudoSlamScan();case"startExecution":return this.triggerExecutionLayer(!0);case"startExecutionKeepMemory":return this.triggerExecutionLayer(!1);case"runBindPathTest":return this.triggerBindPathDirectTest();default:this.report(`未识别的任务动作: ${e}`,"warn")}}publishWorkspaceQuad(){var r,s,o,a;const e=this.rosConnection.getResources(),t=this.workspaceView.getSelectedPoints();if(!(e!=null&&e.workspaceQuadPublisher)||!(e!=null&&e.runWorkspaceS2Publisher)||t.length!==4){this.report("当前还不能提交工作区四边形，请先连上 ROS 并点满 4 个角点","warn");return}const n=yc(t);this.workspaceView.setOverlaySource("s2"),this.workspaceView.setS2OverlayMessage(null),e.workspaceQuadPublisher.publish(new Ze.Message({data:n})),(s=(r=this.callbacks).onResultMessage)==null||s.call(r,`工作区四边形已发送，正在触发 S2: [${n.join(", ")}]`),(a=(o=this.callbacks).onLog)==null||a.call(o,`已发送工作区四边形: [${n.join(", ")}]`,"success"),window.setTimeout(()=>{e.runWorkspaceS2Publisher.publish(new Ze.Message({data:!0}))},180)}triggerSavedWorkspaceS2(){var n,r,s,o;const e=this.rosConnection.getResources(),t=this.workspaceView.getSavedWorkspacePoints();if(!(e!=null&&e.runWorkspaceS2Publisher)||t.length!==4){this.report("当前没有可复用的已保存工作区，请先点 4 个角点并提交","warn");return}this.workspaceView.setOverlaySource("s2"),this.workspaceView.setS2OverlayMessage(null),e.runWorkspaceS2Publisher.publish(new Ze.Message({data:!0})),(r=(n=this.callbacks).onResultMessage)==null||r.call(n,"正在使用当前已保存工作区识别绑扎点..."),(o=(s=this.callbacks).onLog)==null||o.call(s,"已触发基于已保存工作区的 S2 识别","success")}triggerPseudoSlamScan(){var t,n,r,s;const e=this.rosConnection.getResources();if(!(e!=null&&e.startPseudoSlamScanActionClient)){this.report("ROS 还没连好，暂时不能开始固定扫描规划","warn");return}this.workspaceView.setOverlaySource("s2"),this.workspaceView.setS2OverlayMessage(null),(n=(t=this.callbacks).onResultMessage)==null||n.call(t,"正在执行固定工作区扫描：移动到 x=-260, y=1700, z=2997, speed=100，然后触发 S2 并动态规划。"),(s=(r=this.callbacks).onLog)==null||s.call(r,"已触发固定扫描建图任务","success"),this.sendActionGoal(e.startPseudoSlamScanActionClient,{goalMessage:{enable_capture_gate:!1,scan_strategy:2},feedbackPrefix:"扫描建图进行中",successPrefix:"扫描建图完成",failurePrefix:"扫描建图失败"})}triggerExecutionLayer(e){var r,s,o,a;const t=this.rosConnection.getResources();if(!(t!=null&&t.executionModeService)||!(t!=null&&t.startGlobalWorkActionClient)){this.report("ROS 还没连好，暂时不能开始执行层","warn");return}this.workspaceView.setOverlaySource("execution"),this.workspaceView.setExecutionOverlayMessage(null),(s=(r=this.callbacks).onResultMessage)==null||s.call(r,"执行层已切到 result_img 覆盖层，后续显示 /pointAI/result_image_raw。"),(a=(o=this.callbacks).onLog)==null||a.call(o,e?"准备清记忆并开始执行层":"准备保留记忆直接开始执行层","success");const n=new Ze.ServiceRequest({execution_mode:1});t.executionModeService.callService(n,l=>{if(!(l!=null&&l.success)){this.report(`执行模式切换失败: ${(l==null?void 0:l.message)||"未知错误"}`,"error");return}this.sendActionGoal(t.startGlobalWorkActionClient,{goalMessage:{clear_execution_memory:e,execution_mode:1},feedbackPrefix:"执行层进行中",successPrefix:"执行层任务完成",failurePrefix:"执行层任务失败"})},l=>this.report(`执行模式切换失败: ${(l==null?void 0:l.message)||String(l)}`,"error"))}triggerBindPathDirectTest(){var t,n,r,s;const e=this.rosConnection.getResources();if(!(e!=null&&e.runDirectBindPathTestActionClient)){this.report("ROS 还没连好，暂时不能直接执行账本测试","warn");return}(n=(t=this.callbacks).onResultMessage)==null||n.call(t,"直接执行账本测试已触发：后端将只按 pseudo_slam_bind_path.json 的 path_origin、cabin_pose 和 x/y/z 执行。"),(s=(r=this.callbacks).onLog)==null||s.call(r,"已触发直接执行账本测试","success"),this.sendActionGoal(e.runDirectBindPathTestActionClient,{goalMessage:{},feedbackPrefix:"账本测试进行中",successPrefix:"账本测试完成",failurePrefix:"账本测试失败"})}sendActionGoal(e,{goalMessage:t,feedbackPrefix:n,successPrefix:r,failurePrefix:s}){const o=new Ze.Goal({actionClient:e,goalMessage:t});o.on("feedback",a=>{var c,u;const l=(a==null?void 0:a.detail)||(a==null?void 0:a.stage)||"处理中";(u=(c=this.callbacks).onResultMessage)==null||u.call(c,`${n}: ${l}`)}),o.on("result",a=>{var c,u,h,d,f,g,x,m;const l=(a==null?void 0:a.message)||"未知结果";if(a!=null&&a.success){(u=(c=this.callbacks).onResultMessage)==null||u.call(c,`${r}: ${l}`),(d=(h=this.callbacks).onLog)==null||d.call(h,`${r}: ${l}`,"success");return}(g=(f=this.callbacks).onResultMessage)==null||g.call(f,`${s}: ${l}`),(m=(x=this.callbacks).onLog)==null||m.call(x,`${s}: ${l}`,"error")}),o.send()}report(e,t="info"){var n,r,s,o;(r=(n=this.callbacks).onResultMessage)==null||r.call(n,e),(o=(s=this.callbacks).onLog)==null||o.call(s,e,t)}}const Mc=[{id:"onlyPointCloud",label:"只显示点云"},{id:"onlyTiePoints",label:"只显示绑扎点"},{id:"pointCloudAndTiePoints",label:"点云 + 绑扎点"},{id:"planningFocus",label:"规划点/执行点"},{id:"machineOnly",label:"只显示机器"},{id:"all",label:"全开"}],bc=[{id:"filteredWorldCoord",label:"滤波世界点云"},{id:"rawWorldCoord",label:"原始世界点云"}],Ec=[{id:"camera",label:"相机视角"},{id:"global",label:"全局视角"}];function va(i,e,t=e){var n;return((n=i.find(r=>r.id===e))==null?void 0:n.label)||t}const Eo={onlyPointCloud:{showRobot:!1,showAxes:!1,showPointCloud:!0,showTiePoints:!1,showPlanningMarkers:!1},onlyTiePoints:{showRobot:!1,showAxes:!0,showPointCloud:!1,showTiePoints:!0,showPlanningMarkers:!1},pointCloudAndTiePoints:{showRobot:!0,showAxes:!0,showPointCloud:!0,showTiePoints:!0,showPlanningMarkers:!1},planningFocus:{showRobot:!0,showAxes:!0,showPointCloud:!1,showTiePoints:!1,showPlanningMarkers:!0},machineOnly:{showRobot:!0,showAxes:!0,showPointCloud:!1,showTiePoints:!1,showPlanningMarkers:!1},all:{showRobot:!0,showAxes:!0,showPointCloud:!0,showTiePoints:!0,showPlanningMarkers:!0}},Eh={mode:"pointCloudAndTiePoints",pointCloudSource:"filteredWorldCoord",showRobot:!0,showAxes:!0,showPointCloud:!0,showTiePoints:!0,showPlanningMarkers:!1,pointSize:.035,pointOpacity:.78,viewMode:"camera",followCamera:!1};function wh(i,e){const t=Eo[i]||Eo.pointCloudAndTiePoints;return{...e,mode:i,...t}}function wc(i){return va(Mc,i)}function Th(i){return va(bc,i)}function Ah(i){return va(Ec,i)}class Ch{constructor({ui:e,sceneView:t,callbacks:n={}}){this.ui=e,this.sceneView=t,this.callbacks=n,this.state={...Eh},this.stats={filteredWorldCoordCount:0,rawWorldCoordCount:0,tiePointCount:0,planningPointCount:0,tfFrameCount:0}}init(){this.ui.setTopicLayerState(this.state),this.ui.renderTopicLayerStats(this.stats),this.sceneView.setLayerState(this.state),this.sceneView.setViewMode(this.state.viewMode),this.sceneView.setFollowCamera(this.state.followCamera)}handleLayerControlsChange(e){var r,s;const t=e.mode||this.state.mode;let n={...this.state,...e};e.mode&&e.mode!==this.state.mode&&(n=wh(t,n)),this.state=n,this.ui.setTopicLayerState(this.state),this.sceneView.setLayerState(this.state),(s=(r=this.callbacks).onLog)==null||s.call(r,`三维图层模式已切换为 ${wc(this.state.mode)}`,"info")}handleSceneControlsChange(e){var t,n;this.state={...this.state,...e},this.ui.setTopicLayerState(this.state),this.sceneView.setViewMode(this.state.viewMode),this.sceneView.setFollowCamera(this.state.followCamera),(n=(t=this.callbacks).onLog)==null||n.call(t,`三维视角已切换为 ${Ah(this.state.viewMode)}${this.state.followCamera?"（跟随相机）":""}`,"info")}updateStats(e){this.stats={...this.stats,...e},this.ui.renderTopicLayerStats(this.stats)}getState(){return{...this.state}}}class Rh{constructor(){this.panels=new Map}registerPanel(e){if(!e)return;const t=e.querySelector(".panel-header");if(!t)return;let n=!1,r=0,s=0,o=0,a=0;const l=()=>{let h=20;this.panels.forEach(d=>{h=Math.max(h,Number(d.style.zIndex||20))}),e.style.zIndex=String(h+1)},c=h=>{if(!n)return;const d=h.clientX-o,f=h.clientY-a;e.style.left=`${r+d}px`,e.style.top=`${s+f}px`,e.style.right="auto"},u=()=>{n=!1,document.removeEventListener("pointermove",c),document.removeEventListener("pointerup",u)};t.addEventListener("pointerdown",h=>{if(window.innerWidth<=1100||h.target.closest("button, a, input, select, label"))return;n=!0,l();const f=e.getBoundingClientRect();r=f.left,s=f.top,o=h.clientX,a=h.clientY,e.style.left=`${f.left}px`,e.style.top=`${f.top}px`,e.style.right="auto",document.addEventListener("pointermove",c),document.addEventListener("pointerup",u)}),this.panels.set(e.id,e)}init(e){e.querySelectorAll(".floating-panel").forEach(t=>this.registerPanel(t))}}const wo=[{id:"workspacePanel",label:"工作区"},{id:"processPanel",label:"任务"},{id:"topicLayersPanel",label:"话题图层"},{id:"statusPanel",label:"状态"},{id:"logPanel",label:"日志"},{id:"legacyPanel",label:"命令"}],Ph={workspacePanel:!0,processPanel:!0,topicLayersPanel:!0,statusPanel:!0,logPanel:!1,legacyPanel:!0},Lh={auto:"自动增强",strong:"强增强",raw:"原图"};class Uh{constructor(e){this.rootElement=e,this.refs={}}renderShell(){this.rootElement.innerHTML=`
      <div class="app-shell">
        <div id="sceneBackground" class="scene-background"></div>
        <div class="scene-overlay"></div>

        <header class="top-toolbar">
          <div class="toolbar-group toolbar-brand-group">
            <div class="toolbar-brand">绑扎机器人控制台</div>
            <div class="toolbar-brand-subtitle">参考 robot_viewer 布局，三维背景总览作为整页背景</div>
          </div>

          <div class="toolbar-group">
            ${wo.map(e=>`
              <button
                class="toolbar-pill active"
                type="button"
                data-toolbar-action="toggle-panel:${e.id}"
                data-panel-toggle="${e.id}"
              >${e.label}</button>
            `).join("")}
          </div>

          <div class="toolbar-group">
            <button class="toolbar-pill active" type="button" data-toolbar-action="view:camera" data-view-mode="camera">相机视角</button>
            <button class="toolbar-pill" type="button" data-toolbar-action="view:global" data-view-mode="global">全局视角</button>
            <button class="toolbar-pill" type="button" data-toolbar-action="toggle-follow-camera" data-follow-toggle="true">跟随相机</button>
            <button class="toolbar-pill active" type="button" data-toolbar-action="toggle-layer:pointCloud" data-quick-layer="pointCloud">点云</button>
            <button class="toolbar-pill active" type="button" data-toolbar-action="toggle-layer:tiePoints" data-quick-layer="tiePoints">绑扎点</button>
          </div>

          <div class="toolbar-group toolbar-link-group">
            <a class="toolbar-pill toolbar-link" href="/help/" target="_blank" rel="noreferrer">帮助</a>
            <a class="toolbar-pill toolbar-link" href="/help/camera-sdk/index" target="_blank" rel="noreferrer">相机 SDK</a>
          </div>
        </header>

        <div class="floating-panels">
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

          <section id="processPanel" class="floating-panel panel-process">
            <div class="panel-header">
              <div>
                <div class="panel-title">任务</div>
                <div class="panel-subtitle">扫描、识别、执行主链</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:processPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <div class="info-block">
                <div><strong>ROS：</strong><span id="connectionText" class="mono">-</span></div>
                <div id="connectionStatus" class="hint">初始化中...</div>
              </div>

              <div class="section-title">快速任务</div>
              <div class="button-grid">
                <button class="primary-btn" data-task-action="submitQuad" disabled>提交四边形并触发 S2</button>
                <button class="secondary-btn" data-task-action="runSavedS2" disabled>直接识别绑扎点</button>
                <button class="secondary-btn" data-task-action="scanPlan" disabled>固定扫描位姿并规划</button>
                <button class="secondary-btn" data-task-action="startExecution" disabled>开始执行层</button>
                <button class="secondary-btn" data-task-action="startExecutionKeepMemory" disabled>保留记忆开始</button>
                <button class="secondary-btn" data-task-action="runBindPathTest" disabled>直接执行账本测试</button>
              </div>

              <div class="section-title">帮助入口</div>
              <div class="button-row">
                <a class="help-link" href="/help/" target="_blank" rel="noreferrer">工程帮助站</a>
                <a class="help-link" href="/help/camera-sdk/index" target="_blank" rel="noreferrer">相机 SDK 文档</a>
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
                    ${Ec.map(e=>`
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
                    ${Mc.map(e=>`
                      <option value="${e.id}">${e.label}</option>
                    `).join("")}
                  </select>
                </div>
                <div class="field">
                  <label for="pointCloudSource">点云源</label>
                  <select id="pointCloudSource">
                    ${bc.map(e=>`
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

          <section id="statusPanel" class="floating-panel panel-status">
            <div class="panel-header">
              <div>
                <div class="panel-title">状态</div>
                <div class="panel-subtitle">运行状态与结果反馈</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:statusPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <div class="section-title">状态灯</div>
              <div id="statusChips"></div>
              <div class="section-title">任务消息</div>
              <div id="resultMessage" class="info-block hint">等待任务触发。</div>
            </div>
          </section>

          <section id="logPanel" class="floating-panel panel-log">
            <div class="panel-header">
              <div>
                <div class="panel-title">日志</div>
                <div class="panel-subtitle">操作与状态流转日志</div>
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

          <section id="legacyPanel" class="floating-panel panel-legacy" data-size="wide">
            <div class="panel-header">
              <div>
                <div class="panel-title">命令</div>
                <div class="panel-subtitle">参考旧 APP 的常用操作与参数</div>
              </div>
              <div class="panel-actions">
                <button class="panel-action-btn" type="button" data-toolbar-action="toggle-panel:legacyPanel" title="隐藏">✕</button>
              </div>
            </div>
            <div class="panel-content">
              <div class="section-title">参数区</div>
              <div id="parameterFields" class="field-grid"></div>
              <div class="section-title">命令区</div>
              <div id="legacyCommandGroups"></div>
            </div>
          </section>
        </div>
      </div>
    `,this.bindRefs(),this.applyDefaultPanelVisibility(),this.renderStatusChips(),this.renderParameterFields(),this.renderLegacyCommands(),this.renderPointList([]),this.renderLogs([]),this.renderTopicLayerStats({filteredWorldCoordCount:0,rawWorldCoordCount:0,tiePointCount:0,planningPointCount:0,tfFrameCount:0})}bindRefs(){this.refs.sceneBackground=this.rootElement.querySelector("#sceneBackground"),this.refs.irCanvas=this.rootElement.querySelector("#irCanvas"),this.refs.overlayCanvas=this.rootElement.querySelector("#overlayCanvas"),this.refs.sceneViewMode=this.rootElement.querySelector("#sceneViewMode"),this.refs.followCameraToggle=this.rootElement.querySelector("#followCameraToggle"),this.refs.topicLayerMode=this.rootElement.querySelector("#topicLayerMode"),this.refs.pointCloudSource=this.rootElement.querySelector("#pointCloudSource"),this.refs.showRobotToggle=this.rootElement.querySelector("#showRobotToggle"),this.refs.showAxesToggle=this.rootElement.querySelector("#showAxesToggle"),this.refs.showPointCloudToggle=this.rootElement.querySelector("#showPointCloudToggle"),this.refs.showTiePointsToggle=this.rootElement.querySelector("#showTiePointsToggle"),this.refs.showPlanningMarkersToggle=this.rootElement.querySelector("#showPlanningMarkersToggle"),this.refs.pointSizeRange=this.rootElement.querySelector("#pointSizeRange"),this.refs.pointOpacityRange=this.rootElement.querySelector("#pointOpacityRange"),this.refs.topicLayerSummary=this.rootElement.querySelector("#topicLayerSummary"),this.refs.topicLayerStats=this.rootElement.querySelector("#topicLayerStats"),this.refs.connectionText=this.rootElement.querySelector("#connectionText"),this.refs.connectionStatus=this.rootElement.querySelector("#connectionStatus"),this.refs.selectedPoints=this.rootElement.querySelector("#selectedPoints"),this.refs.displayMode=this.rootElement.querySelector("#displayMode"),this.refs.gammaRange=this.rootElement.querySelector("#gammaRange"),this.refs.overlayOpacityRange=this.rootElement.querySelector("#overlayOpacityRange"),this.refs.displaySettingSummary=this.rootElement.querySelector("#displaySettingSummary"),this.refs.statusChips=this.rootElement.querySelector("#statusChips"),this.refs.resultMessage=this.rootElement.querySelector("#resultMessage"),this.refs.logList=this.rootElement.querySelector("#logList"),this.refs.clearLogs=this.rootElement.querySelector("#clearLogs"),this.refs.parameterFields=this.rootElement.querySelector("#parameterFields"),this.refs.legacyCommandGroups=this.rootElement.querySelector("#legacyCommandGroups"),this.refs.taskButtons=[...this.rootElement.querySelectorAll("[data-task-action]")],this.refs.workspaceButtons=[...this.rootElement.querySelectorAll("[data-workspace-action]")],this.refs.toolbarButtons=[...this.rootElement.querySelectorAll("[data-toolbar-action]")],this.refs.panelElements=new Map(wo.map(({id:e})=>[e,this.rootElement.querySelector(`#${e}`)])),this.refs.panelToggleButtons=[...this.rootElement.querySelectorAll("[data-panel-toggle]")],this.refs.topicLayerInputs=[this.refs.topicLayerMode,this.refs.pointCloudSource,this.refs.showRobotToggle,this.refs.showAxesToggle,this.refs.showPointCloudToggle,this.refs.showTiePointsToggle,this.refs.showPlanningMarkersToggle,this.refs.pointSizeRange,this.refs.pointOpacityRange],this.refs.sceneInputs=[this.refs.sceneViewMode,this.refs.followCameraToggle]}applyDefaultPanelVisibility(){Object.entries(Ph).forEach(([e,t])=>{this.setPanelVisible(e,t)})}renderStatusChips(){this.refs.statusChips.innerHTML=xc.map(e=>`
      <span class="status-chip info" data-status-id="${e.id}">${e.label}</span>
    `).join("")}renderParameterFields(){this.refs.parameterFields.innerHTML=Object.entries(Lr).map(([e,t])=>`
        <div class="field">
          <label for="param-${e}">${fu[e]||e}</label>
          <input id="param-${e}" name="${e}" type="number" step="0.1" value="${t}" />
        </div>
      `).join("")}renderLegacyCommands(){const e=new Map;sc.forEach(t=>{e.has(t.group)||e.set(t.group,[]),e.get(t.group).push(t)}),this.refs.legacyCommandGroups.innerHTML=[...e.entries()].map(([t,n])=>`
      <div class="legacy-group">
        <div class="section-title">${t}</div>
        <div class="button-grid">
          ${n.map(r=>`
            <button class="secondary-btn" data-legacy-command="${r.id}" type="button">
              ${r.name}
            </button>
          `).join("")}
        </div>
      </div>
    `).join(""),this.refs.legacyButtons=[...this.rootElement.querySelectorAll("[data-legacy-command]")]}getCanvasRefs(){return{canvas:this.refs.irCanvas,overlayCanvas:this.refs.overlayCanvas}}getSceneContainer(){return this.refs.sceneBackground}getDisplayControls(){return{displayMode:this.refs.displayMode,gammaRange:this.refs.gammaRange,overlayOpacityRange:this.refs.overlayOpacityRange}}getParameterValues(){return Object.keys(Lr).reduce((e,t)=>{const n=this.rootElement.querySelector(`#param-${t}`);return e[t]=Number.parseFloat((n==null?void 0:n.value)??Lr[t]),e},{})}getTopicLayerState(){return{mode:this.refs.topicLayerMode.value,pointCloudSource:this.refs.pointCloudSource.value,showRobot:this.refs.showRobotToggle.checked,showAxes:this.refs.showAxesToggle.checked,showPointCloud:this.refs.showPointCloudToggle.checked,showTiePoints:this.refs.showTiePointsToggle.checked,showPlanningMarkers:this.refs.showPlanningMarkersToggle.checked,pointSize:Number.parseFloat(this.refs.pointSizeRange.value),pointOpacity:Number.parseFloat(this.refs.pointOpacityRange.value),viewMode:this.refs.sceneViewMode.value,followCamera:this.refs.followCameraToggle.checked}}getSceneViewState(){return{viewMode:this.refs.sceneViewMode.value,followCamera:this.refs.followCameraToggle.checked}}isPanelVisible(e){const t=this.refs.panelElements.get(e);return!!(t&&t.dataset.visible!=="false")}setPanelVisible(e,t){const n=this.refs.panelElements.get(e);n&&(n.hidden=!t,n.dataset.visible=t?"true":"false",n.style.display=t?"":"none",this.refs.panelToggleButtons.filter(r=>r.dataset.panelToggle===e).forEach(r=>{r.classList.toggle("active",t),r.setAttribute("aria-pressed",t?"true":"false")}))}togglePanelVisible(e){const t=!this.isPanelVisible(e);return this.setPanelVisible(e,t),t}setToolbarSceneState(e){this.rootElement.querySelectorAll("[data-view-mode]").forEach(n=>{const r=n.dataset.viewMode===e.viewMode;n.classList.toggle("active",r),n.setAttribute("aria-pressed",r?"true":"false")});const t=this.rootElement.querySelector("[data-follow-toggle]");t&&(t.classList.toggle("active",!!e.followCamera),t.setAttribute("aria-pressed",e.followCamera?"true":"false")),this.rootElement.querySelectorAll("[data-quick-layer]").forEach(n=>{const r=n.dataset.quickLayer,s=r==="pointCloud"&&!!e.showPointCloud||r==="tiePoints"&&!!e.showTiePoints;n.classList.toggle("active",s),n.setAttribute("aria-pressed",s?"true":"false")})}onToolbarAction(e){this.refs.toolbarButtons.forEach(t=>{t.addEventListener("click",n=>{n.preventDefault(),n.stopPropagation(),e(t.dataset.toolbarAction)})})}onTaskAction(e){this.refs.taskButtons.forEach(t=>{t.addEventListener("click",()=>e(t.dataset.taskAction))})}onWorkspaceAction(e){this.refs.workspaceButtons.forEach(t=>{t.addEventListener("click",()=>e(t.dataset.workspaceAction))})}onLegacyCommand(e){this.refs.legacyButtons.forEach(t=>{t.addEventListener("click",()=>e(Number(t.dataset.legacyCommand)))})}onDisplaySettingsChange(e){[this.refs.displayMode,this.refs.gammaRange,this.refs.overlayOpacityRange].forEach(t=>{t.addEventListener("input",()=>e(this.getDisplaySettings())),t.addEventListener("change",()=>e(this.getDisplaySettings()))})}onSceneControlsChange(e){this.refs.sceneInputs.forEach(t=>{const n=t.type==="checkbox"?"change":"input";t.addEventListener(n,()=>e(this.getSceneViewState()))})}onTopicLayerControlsChange(e){this.refs.topicLayerInputs.forEach(t=>{const n=t.type==="checkbox"?"change":"input";t.addEventListener(n,()=>e(this.getTopicLayerState())),n!=="change"&&t.addEventListener("change",()=>e(this.getTopicLayerState()))})}onClearLogs(e){this.refs.clearLogs.addEventListener("click",e)}setConnectionInfo(e,t,n="info"){this.refs.connectionText.textContent=e||"-",this.refs.connectionStatus.textContent=t,this.refs.connectionStatus.className=`hint ${n}`}setTaskButtonsEnabled(e){this.refs.taskButtons.forEach(t=>{const n=t.dataset.taskAction;t.disabled=e[n]===!1})}setWorkspaceButtonsEnabled(e){this.refs.workspaceButtons.forEach(t=>{const n=t.dataset.workspaceAction;t.disabled=e[n]===!1})}setDisplaySettings(e){this.refs.displayMode.value=e.mode,this.refs.gammaRange.value=e.gamma.toFixed(2),this.refs.overlayOpacityRange.value=e.overlayOpacity.toFixed(2),this.refs.displaySettingSummary.textContent=`模式=${Lh[e.mode]||e.mode} 伽马=${e.gamma.toFixed(2)} 覆盖=${e.overlayOpacity.toFixed(2)}`}setTopicLayerState(e){this.refs.topicLayerMode.value=e.mode,this.refs.pointCloudSource.value=e.pointCloudSource,this.refs.showRobotToggle.checked=!!e.showRobot,this.refs.showAxesToggle.checked=!!e.showAxes,this.refs.showPointCloudToggle.checked=!!e.showPointCloud,this.refs.showTiePointsToggle.checked=!!e.showTiePoints,this.refs.showPlanningMarkersToggle.checked=!!e.showPlanningMarkers,this.refs.pointSizeRange.value=Number(e.pointSize).toFixed(3),this.refs.pointOpacityRange.value=Number(e.pointOpacity).toFixed(2),this.refs.sceneViewMode.value=e.viewMode,this.refs.followCameraToggle.checked=!!e.followCamera,this.refs.topicLayerSummary.textContent=`模式=${wc(e.mode)} 点云源=${Th(e.pointCloudSource)} 点大小=${Number(e.pointSize).toFixed(3)} 透明度=${Number(e.pointOpacity).toFixed(2)}`,this.setToolbarSceneState(e)}renderTopicLayerStats(e){this.refs.topicLayerStats.innerHTML=`
      <div class="stats-card"><span>滤波点云</span><strong>${e.filteredWorldCoordCount}</strong></div>
      <div class="stats-card"><span>原始点云</span><strong>${e.rawWorldCoordCount}</strong></div>
      <div class="stats-card"><span>绑扎点</span><strong>${e.tiePointCount}</strong></div>
      <div class="stats-card"><span>规划点</span><strong>${e.planningPointCount}</strong></div>
      <div class="stats-card"><span>TF 帧</span><strong>${e.tfFrameCount}</strong></div>
    `}getDisplaySettings(){return{mode:this.refs.displayMode.value,gamma:Number.parseFloat(this.refs.gammaRange.value),overlayOpacity:Number.parseFloat(this.refs.overlayOpacityRange.value)}}renderPointList(e){if(!e.length){this.refs.selectedPoints.innerHTML='<li class="point-item">还没有点，直接在 IR 图上点 4 个角点。</li>';return}this.refs.selectedPoints.innerHTML=e.map((t,n)=>`
      <li class="point-item mono">${n+1}. x=${t.x}, y=${t.y}</li>
    `).join("")}setStatusChipState(e,t,n){const r=this.rootElement.querySelector(`[data-status-id="${e}"]`);r&&(r.className=`status-chip ${t}`,r.title=n||"")}setResultMessage(e){this.refs.resultMessage.textContent=e}renderLogs(e){if(!e.length){this.refs.logList.innerHTML='<li class="log-item info">暂无日志，等待前端动作或 ROS 状态变化。</li>';return}this.refs.logList.innerHTML=e.map(t=>`
      <li class="log-item ${t.level}">
        <div class="log-meta mono">${t.timestamp}</div>
        <div>${t.message}</div>
      </li>
    `).join("")}}const Tc="tie_robot_frontend_display_preferences";function Dh(){const i={mode:"auto",gamma:.85,overlayOpacity:.88};try{const e=localStorage.getItem(Tc);if(!e)return i;const t=JSON.parse(e);return{mode:["raw","auto","strong"].includes(t==null?void 0:t.mode)?t.mode:i.mode,gamma:Number.isFinite(t==null?void 0:t.gamma)?t.gamma:i.gamma,overlayOpacity:Number.isFinite(t==null?void 0:t.overlayOpacity)?t.overlayOpacity:i.overlayOpacity}}catch{return i}}function Ih(i){try{localStorage.setItem(Tc,JSON.stringify(i))}catch{}}/**
 * @license
 * Copyright 2010-2024 Three.js Authors
 * SPDX-License-Identifier: MIT
 */const xa="165",kn={ROTATE:0,DOLLY:1,PAN:2},zn={ROTATE:0,PAN:1,DOLLY_PAN:2,DOLLY_ROTATE:3},Nh=0,To=1,Oh=2,Ac=1,Fh=2,Kt=3,pn=0,yt=1,Zt=2,dn=0,si=1,Ao=2,Co=3,Ro=4,Bh=5,Cn=100,kh=101,zh=102,Vh=103,Gh=104,Hh=200,Wh=201,Xh=202,qh=203,sa=204,aa=205,Yh=206,jh=207,$h=208,Kh=209,Zh=210,Jh=211,Qh=212,ed=213,td=214,nd=0,id=1,rd=2,dr=3,sd=4,ad=5,od=6,ld=7,Cc=0,cd=1,ud=2,fn=0,hd=1,dd=2,fd=3,pd=4,md=5,gd=6,_d=7,Rc=300,li=301,ci=302,oa=303,la=304,Mr=306,ca=1e3,Pn=1001,ua=1002,Pt=1003,vd=1004,Ni=1005,Nt=1006,bs=1007,Ln=1008,mn=1009,xd=1010,yd=1011,fr=1012,Pc=1013,ui=1014,un=1015,br=1016,Lc=1017,Uc=1018,hi=1020,Sd=35902,Md=1021,bd=1022,zt=1023,Ed=1024,wd=1025,ai=1026,di=1027,Td=1028,Dc=1029,Ad=1030,Ic=1031,Nc=1033,Es=33776,ws=33777,Ts=33778,As=33779,Po=35840,Lo=35841,Uo=35842,Do=35843,Io=36196,No=37492,Oo=37496,Fo=37808,Bo=37809,ko=37810,zo=37811,Vo=37812,Go=37813,Ho=37814,Wo=37815,Xo=37816,qo=37817,Yo=37818,jo=37819,$o=37820,Ko=37821,Cs=36492,Zo=36494,Jo=36495,Cd=36283,Qo=36284,el=36285,tl=36286,Rd=3200,Pd=3201,Oc=0,Ld=1,cn="",Ft="srgb",_n="srgb-linear",ya="display-p3",Er="display-p3-linear",pr="linear",Ke="srgb",mr="rec709",gr="p3",Vn=7680,nl=519,Ud=512,Dd=513,Id=514,Fc=515,Nd=516,Od=517,Fd=518,Bd=519,il=35044,rl="300 es",Jt=2e3,_r=2001;class Fn{addEventListener(e,t){this._listeners===void 0&&(this._listeners={});const n=this._listeners;n[e]===void 0&&(n[e]=[]),n[e].indexOf(t)===-1&&n[e].push(t)}hasEventListener(e,t){if(this._listeners===void 0)return!1;const n=this._listeners;return n[e]!==void 0&&n[e].indexOf(t)!==-1}removeEventListener(e,t){if(this._listeners===void 0)return;const r=this._listeners[e];if(r!==void 0){const s=r.indexOf(t);s!==-1&&r.splice(s,1)}}dispatchEvent(e){if(this._listeners===void 0)return;const n=this._listeners[e.type];if(n!==void 0){e.target=this;const r=n.slice(0);for(let s=0,o=r.length;s<o;s++)r[s].call(this,e);e.target=null}}}const pt=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"],ur=Math.PI/180,ha=180/Math.PI;function Pi(){const i=Math.random()*4294967295|0,e=Math.random()*4294967295|0,t=Math.random()*4294967295|0,n=Math.random()*4294967295|0;return(pt[i&255]+pt[i>>8&255]+pt[i>>16&255]+pt[i>>24&255]+"-"+pt[e&255]+pt[e>>8&255]+"-"+pt[e>>16&15|64]+pt[e>>24&255]+"-"+pt[t&63|128]+pt[t>>8&255]+"-"+pt[t>>16&255]+pt[t>>24&255]+pt[n&255]+pt[n>>8&255]+pt[n>>16&255]+pt[n>>24&255]).toLowerCase()}function vt(i,e,t){return Math.max(e,Math.min(t,i))}function kd(i,e){return(i%e+e)%e}function Rs(i,e,t){return(1-t)*i+t*e}function vi(i,e){switch(e.constructor){case Float32Array:return i;case Uint32Array:return i/4294967295;case Uint16Array:return i/65535;case Uint8Array:return i/255;case Int32Array:return Math.max(i/2147483647,-1);case Int16Array:return Math.max(i/32767,-1);case Int8Array:return Math.max(i/127,-1);default:throw new Error("Invalid component type.")}}function xt(i,e){switch(e.constructor){case Float32Array:return i;case Uint32Array:return Math.round(i*4294967295);case Uint16Array:return Math.round(i*65535);case Uint8Array:return Math.round(i*255);case Int32Array:return Math.round(i*2147483647);case Int16Array:return Math.round(i*32767);case Int8Array:return Math.round(i*127);default:throw new Error("Invalid component type.")}}const zd={DEG2RAD:ur};class De{constructor(e=0,t=0){De.prototype.isVector2=!0,this.x=e,this.y=t}get width(){return this.x}set width(e){this.x=e}get height(){return this.y}set height(e){this.y=e}set(e,t){return this.x=e,this.y=t,this}setScalar(e){return this.x=e,this.y=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y)}copy(e){return this.x=e.x,this.y=e.y,this}add(e){return this.x+=e.x,this.y+=e.y,this}addScalar(e){return this.x+=e,this.y+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this}subScalar(e){return this.x-=e,this.y-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this}multiply(e){return this.x*=e.x,this.y*=e.y,this}multiplyScalar(e){return this.x*=e,this.y*=e,this}divide(e){return this.x/=e.x,this.y/=e.y,this}divideScalar(e){return this.multiplyScalar(1/e)}applyMatrix3(e){const t=this.x,n=this.y,r=e.elements;return this.x=r[0]*t+r[3]*n+r[6],this.y=r[1]*t+r[4]*n+r[7],this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this}clampLength(e,t){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(e,Math.min(t,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(e){return this.x*e.x+this.y*e.y}cross(e){return this.x*e.y-this.y*e.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const n=this.dot(e)/t;return Math.acos(vt(n,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,n=this.y-e.y;return t*t+n*n}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this}equals(e){return e.x===this.x&&e.y===this.y}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this}rotateAround(e,t){const n=Math.cos(t),r=Math.sin(t),s=this.x-e.x,o=this.y-e.y;return this.x=s*n-o*r+e.x,this.y=s*r+o*n+e.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class ze{constructor(e,t,n,r,s,o,a,l,c){ze.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],e!==void 0&&this.set(e,t,n,r,s,o,a,l,c)}set(e,t,n,r,s,o,a,l,c){const u=this.elements;return u[0]=e,u[1]=r,u[2]=a,u[3]=t,u[4]=s,u[5]=l,u[6]=n,u[7]=o,u[8]=c,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(e){const t=this.elements,n=e.elements;return t[0]=n[0],t[1]=n[1],t[2]=n[2],t[3]=n[3],t[4]=n[4],t[5]=n[5],t[6]=n[6],t[7]=n[7],t[8]=n[8],this}extractBasis(e,t,n){return e.setFromMatrix3Column(this,0),t.setFromMatrix3Column(this,1),n.setFromMatrix3Column(this,2),this}setFromMatrix4(e){const t=e.elements;return this.set(t[0],t[4],t[8],t[1],t[5],t[9],t[2],t[6],t[10]),this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const n=e.elements,r=t.elements,s=this.elements,o=n[0],a=n[3],l=n[6],c=n[1],u=n[4],h=n[7],d=n[2],f=n[5],g=n[8],x=r[0],m=r[3],p=r[6],w=r[1],S=r[4],A=r[7],k=r[2],I=r[5],P=r[8];return s[0]=o*x+a*w+l*k,s[3]=o*m+a*S+l*I,s[6]=o*p+a*A+l*P,s[1]=c*x+u*w+h*k,s[4]=c*m+u*S+h*I,s[7]=c*p+u*A+h*P,s[2]=d*x+f*w+g*k,s[5]=d*m+f*S+g*I,s[8]=d*p+f*A+g*P,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[3]*=e,t[6]*=e,t[1]*=e,t[4]*=e,t[7]*=e,t[2]*=e,t[5]*=e,t[8]*=e,this}determinant(){const e=this.elements,t=e[0],n=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8];return t*o*u-t*a*c-n*s*u+n*a*l+r*s*c-r*o*l}invert(){const e=this.elements,t=e[0],n=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8],h=u*o-a*c,d=a*l-u*s,f=c*s-o*l,g=t*h+n*d+r*f;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);const x=1/g;return e[0]=h*x,e[1]=(r*c-u*n)*x,e[2]=(a*n-r*o)*x,e[3]=d*x,e[4]=(u*t-r*l)*x,e[5]=(r*s-a*t)*x,e[6]=f*x,e[7]=(n*l-c*t)*x,e[8]=(o*t-n*s)*x,this}transpose(){let e;const t=this.elements;return e=t[1],t[1]=t[3],t[3]=e,e=t[2],t[2]=t[6],t[6]=e,e=t[5],t[5]=t[7],t[7]=e,this}getNormalMatrix(e){return this.setFromMatrix4(e).invert().transpose()}transposeIntoArray(e){const t=this.elements;return e[0]=t[0],e[1]=t[3],e[2]=t[6],e[3]=t[1],e[4]=t[4],e[5]=t[7],e[6]=t[2],e[7]=t[5],e[8]=t[8],this}setUvTransform(e,t,n,r,s,o,a){const l=Math.cos(s),c=Math.sin(s);return this.set(n*l,n*c,-n*(l*o+c*a)+o+e,-r*c,r*l,-r*(-c*o+l*a)+a+t,0,0,1),this}scale(e,t){return this.premultiply(Ps.makeScale(e,t)),this}rotate(e){return this.premultiply(Ps.makeRotation(-e)),this}translate(e,t){return this.premultiply(Ps.makeTranslation(e,t)),this}makeTranslation(e,t){return e.isVector2?this.set(1,0,e.x,0,1,e.y,0,0,1):this.set(1,0,e,0,1,t,0,0,1),this}makeRotation(e){const t=Math.cos(e),n=Math.sin(e);return this.set(t,-n,0,n,t,0,0,0,1),this}makeScale(e,t){return this.set(e,0,0,0,t,0,0,0,1),this}equals(e){const t=this.elements,n=e.elements;for(let r=0;r<9;r++)if(t[r]!==n[r])return!1;return!0}fromArray(e,t=0){for(let n=0;n<9;n++)this.elements[n]=e[n+t];return this}toArray(e=[],t=0){const n=this.elements;return e[t]=n[0],e[t+1]=n[1],e[t+2]=n[2],e[t+3]=n[3],e[t+4]=n[4],e[t+5]=n[5],e[t+6]=n[6],e[t+7]=n[7],e[t+8]=n[8],e}clone(){return new this.constructor().fromArray(this.elements)}}const Ps=new ze;function Bc(i){for(let e=i.length-1;e>=0;--e)if(i[e]>=65535)return!0;return!1}function vr(i){return document.createElementNS("http://www.w3.org/1999/xhtml",i)}function Vd(){const i=vr("canvas");return i.style.display="block",i}const sl={};function kc(i){i in sl||(sl[i]=!0,console.warn(i))}function Gd(i,e,t){return new Promise(function(n,r){function s(){switch(i.clientWaitSync(e,i.SYNC_FLUSH_COMMANDS_BIT,0)){case i.WAIT_FAILED:r();break;case i.TIMEOUT_EXPIRED:setTimeout(s,t);break;default:n()}}setTimeout(s,t)})}const al=new ze().set(.8224621,.177538,0,.0331941,.9668058,0,.0170827,.0723974,.9105199),ol=new ze().set(1.2249401,-.2249404,0,-.0420569,1.0420571,0,-.0196376,-.0786361,1.0982735),Oi={[_n]:{transfer:pr,primaries:mr,toReference:i=>i,fromReference:i=>i},[Ft]:{transfer:Ke,primaries:mr,toReference:i=>i.convertSRGBToLinear(),fromReference:i=>i.convertLinearToSRGB()},[Er]:{transfer:pr,primaries:gr,toReference:i=>i.applyMatrix3(ol),fromReference:i=>i.applyMatrix3(al)},[ya]:{transfer:Ke,primaries:gr,toReference:i=>i.convertSRGBToLinear().applyMatrix3(ol),fromReference:i=>i.applyMatrix3(al).convertLinearToSRGB()}},Hd=new Set([_n,Er]),je={enabled:!0,_workingColorSpace:_n,get workingColorSpace(){return this._workingColorSpace},set workingColorSpace(i){if(!Hd.has(i))throw new Error(`Unsupported working color space, "${i}".`);this._workingColorSpace=i},convert:function(i,e,t){if(this.enabled===!1||e===t||!e||!t)return i;const n=Oi[e].toReference,r=Oi[t].fromReference;return r(n(i))},fromWorkingColorSpace:function(i,e){return this.convert(i,this._workingColorSpace,e)},toWorkingColorSpace:function(i,e){return this.convert(i,e,this._workingColorSpace)},getPrimaries:function(i){return Oi[i].primaries},getTransfer:function(i){return i===cn?pr:Oi[i].transfer}};function oi(i){return i<.04045?i*.0773993808:Math.pow(i*.9478672986+.0521327014,2.4)}function Ls(i){return i<.0031308?i*12.92:1.055*Math.pow(i,.41666)-.055}let Gn;class Wd{static getDataURL(e){if(/^data:/i.test(e.src)||typeof HTMLCanvasElement>"u")return e.src;let t;if(e instanceof HTMLCanvasElement)t=e;else{Gn===void 0&&(Gn=vr("canvas")),Gn.width=e.width,Gn.height=e.height;const n=Gn.getContext("2d");e instanceof ImageData?n.putImageData(e,0,0):n.drawImage(e,0,0,e.width,e.height),t=Gn}return t.width>2048||t.height>2048?(console.warn("THREE.ImageUtils.getDataURL: Image converted to jpg for performance reasons",e),t.toDataURL("image/jpeg",.6)):t.toDataURL("image/png")}static sRGBToLinear(e){if(typeof HTMLImageElement<"u"&&e instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&e instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&e instanceof ImageBitmap){const t=vr("canvas");t.width=e.width,t.height=e.height;const n=t.getContext("2d");n.drawImage(e,0,0,e.width,e.height);const r=n.getImageData(0,0,e.width,e.height),s=r.data;for(let o=0;o<s.length;o++)s[o]=oi(s[o]/255)*255;return n.putImageData(r,0,0),t}else if(e.data){const t=e.data.slice(0);for(let n=0;n<t.length;n++)t instanceof Uint8Array||t instanceof Uint8ClampedArray?t[n]=Math.floor(oi(t[n]/255)*255):t[n]=oi(t[n]);return{data:t,width:e.width,height:e.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),e}}let Xd=0;class zc{constructor(e=null){this.isSource=!0,Object.defineProperty(this,"id",{value:Xd++}),this.uuid=Pi(),this.data=e,this.dataReady=!0,this.version=0}set needsUpdate(e){e===!0&&this.version++}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.images[this.uuid]!==void 0)return e.images[this.uuid];const n={uuid:this.uuid,url:""},r=this.data;if(r!==null){let s;if(Array.isArray(r)){s=[];for(let o=0,a=r.length;o<a;o++)r[o].isDataTexture?s.push(Us(r[o].image)):s.push(Us(r[o]))}else s=Us(r);n.url=s}return t||(e.images[this.uuid]=n),n}}function Us(i){return typeof HTMLImageElement<"u"&&i instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&i instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&i instanceof ImageBitmap?Wd.getDataURL(i):i.data?{data:Array.from(i.data),width:i.width,height:i.height,type:i.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let qd=0;class St extends Fn{constructor(e=St.DEFAULT_IMAGE,t=St.DEFAULT_MAPPING,n=Pn,r=Pn,s=Nt,o=Ln,a=zt,l=mn,c=St.DEFAULT_ANISOTROPY,u=cn){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:qd++}),this.uuid=Pi(),this.name="",this.source=new zc(e),this.mipmaps=[],this.mapping=t,this.channel=0,this.wrapS=n,this.wrapT=r,this.magFilter=s,this.minFilter=o,this.anisotropy=c,this.format=a,this.internalFormat=null,this.type=l,this.offset=new De(0,0),this.repeat=new De(1,1),this.center=new De(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new ze,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=u,this.userData={},this.version=0,this.onUpdate=null,this.isRenderTargetTexture=!1,this.pmremVersion=0}get image(){return this.source.data}set image(e=null){this.source.data=e}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(e){return this.name=e.name,this.source=e.source,this.mipmaps=e.mipmaps.slice(0),this.mapping=e.mapping,this.channel=e.channel,this.wrapS=e.wrapS,this.wrapT=e.wrapT,this.magFilter=e.magFilter,this.minFilter=e.minFilter,this.anisotropy=e.anisotropy,this.format=e.format,this.internalFormat=e.internalFormat,this.type=e.type,this.offset.copy(e.offset),this.repeat.copy(e.repeat),this.center.copy(e.center),this.rotation=e.rotation,this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrix.copy(e.matrix),this.generateMipmaps=e.generateMipmaps,this.premultiplyAlpha=e.premultiplyAlpha,this.flipY=e.flipY,this.unpackAlignment=e.unpackAlignment,this.colorSpace=e.colorSpace,this.userData=JSON.parse(JSON.stringify(e.userData)),this.needsUpdate=!0,this}toJSON(e){const t=e===void 0||typeof e=="string";if(!t&&e.textures[this.uuid]!==void 0)return e.textures[this.uuid];const n={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(e).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(n.userData=this.userData),t||(e.textures[this.uuid]=n),n}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(e){if(this.mapping!==Rc)return e;if(e.applyMatrix3(this.matrix),e.x<0||e.x>1)switch(this.wrapS){case ca:e.x=e.x-Math.floor(e.x);break;case Pn:e.x=e.x<0?0:1;break;case ua:Math.abs(Math.floor(e.x)%2)===1?e.x=Math.ceil(e.x)-e.x:e.x=e.x-Math.floor(e.x);break}if(e.y<0||e.y>1)switch(this.wrapT){case ca:e.y=e.y-Math.floor(e.y);break;case Pn:e.y=e.y<0?0:1;break;case ua:Math.abs(Math.floor(e.y)%2)===1?e.y=Math.ceil(e.y)-e.y:e.y=e.y-Math.floor(e.y);break}return this.flipY&&(e.y=1-e.y),e}set needsUpdate(e){e===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(e){e===!0&&this.pmremVersion++}}St.DEFAULT_IMAGE=null;St.DEFAULT_MAPPING=Rc;St.DEFAULT_ANISOTROPY=1;class ht{constructor(e=0,t=0,n=0,r=1){ht.prototype.isVector4=!0,this.x=e,this.y=t,this.z=n,this.w=r}get width(){return this.z}set width(e){this.z=e}get height(){return this.w}set height(e){this.w=e}set(e,t,n,r){return this.x=e,this.y=t,this.z=n,this.w=r,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this.w=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setW(e){return this.w=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;case 3:this.w=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this.w=e.w!==void 0?e.w:1,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this.w+=e.w,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this.w+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this.w=e.w+t.w,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this.w+=e.w*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this.w-=e.w,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this.w-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this.w=e.w-t.w,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this.w*=e.w,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this.w*=e,this}applyMatrix4(e){const t=this.x,n=this.y,r=this.z,s=this.w,o=e.elements;return this.x=o[0]*t+o[4]*n+o[8]*r+o[12]*s,this.y=o[1]*t+o[5]*n+o[9]*r+o[13]*s,this.z=o[2]*t+o[6]*n+o[10]*r+o[14]*s,this.w=o[3]*t+o[7]*n+o[11]*r+o[15]*s,this}divideScalar(e){return this.multiplyScalar(1/e)}setAxisAngleFromQuaternion(e){this.w=2*Math.acos(e.w);const t=Math.sqrt(1-e.w*e.w);return t<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=e.x/t,this.y=e.y/t,this.z=e.z/t),this}setAxisAngleFromRotationMatrix(e){let t,n,r,s;const l=e.elements,c=l[0],u=l[4],h=l[8],d=l[1],f=l[5],g=l[9],x=l[2],m=l[6],p=l[10];if(Math.abs(u-d)<.01&&Math.abs(h-x)<.01&&Math.abs(g-m)<.01){if(Math.abs(u+d)<.1&&Math.abs(h+x)<.1&&Math.abs(g+m)<.1&&Math.abs(c+f+p-3)<.1)return this.set(1,0,0,0),this;t=Math.PI;const S=(c+1)/2,A=(f+1)/2,k=(p+1)/2,I=(u+d)/4,P=(h+x)/4,Y=(g+m)/4;return S>A&&S>k?S<.01?(n=0,r=.707106781,s=.707106781):(n=Math.sqrt(S),r=I/n,s=P/n):A>k?A<.01?(n=.707106781,r=0,s=.707106781):(r=Math.sqrt(A),n=I/r,s=Y/r):k<.01?(n=.707106781,r=.707106781,s=0):(s=Math.sqrt(k),n=P/s,r=Y/s),this.set(n,r,s,t),this}let w=Math.sqrt((m-g)*(m-g)+(h-x)*(h-x)+(d-u)*(d-u));return Math.abs(w)<.001&&(w=1),this.x=(m-g)/w,this.y=(h-x)/w,this.z=(d-u)/w,this.w=Math.acos((c+f+p-1)/2),this}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this.w=Math.min(this.w,e.w),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this.w=Math.max(this.w,e.w),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this.w=Math.max(e.w,Math.min(t.w,this.w)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this.w=Math.max(e,Math.min(t,this.w)),this}clampLength(e,t){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(e,Math.min(t,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z+this.w*e.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this.w+=(e.w-this.w)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this.z=e.z+(t.z-e.z)*n,this.w=e.w+(t.w-e.w)*n,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z&&e.w===this.w}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this.w=e[t+3],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e[t+3]=this.w,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this.w=e.getW(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class Yd extends Fn{constructor(e=1,t=1,n={}){super(),this.isRenderTarget=!0,this.width=e,this.height=t,this.depth=1,this.scissor=new ht(0,0,e,t),this.scissorTest=!1,this.viewport=new ht(0,0,e,t);const r={width:e,height:t,depth:1};n=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:Nt,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1},n);const s=new St(r,n.mapping,n.wrapS,n.wrapT,n.magFilter,n.minFilter,n.format,n.type,n.anisotropy,n.colorSpace);s.flipY=!1,s.generateMipmaps=n.generateMipmaps,s.internalFormat=n.internalFormat,this.textures=[];const o=n.count;for(let a=0;a<o;a++)this.textures[a]=s.clone(),this.textures[a].isRenderTargetTexture=!0;this.depthBuffer=n.depthBuffer,this.stencilBuffer=n.stencilBuffer,this.resolveDepthBuffer=n.resolveDepthBuffer,this.resolveStencilBuffer=n.resolveStencilBuffer,this.depthTexture=n.depthTexture,this.samples=n.samples}get texture(){return this.textures[0]}set texture(e){this.textures[0]=e}setSize(e,t,n=1){if(this.width!==e||this.height!==t||this.depth!==n){this.width=e,this.height=t,this.depth=n;for(let r=0,s=this.textures.length;r<s;r++)this.textures[r].image.width=e,this.textures[r].image.height=t,this.textures[r].image.depth=n;this.dispose()}this.viewport.set(0,0,e,t),this.scissor.set(0,0,e,t)}clone(){return new this.constructor().copy(this)}copy(e){this.width=e.width,this.height=e.height,this.depth=e.depth,this.scissor.copy(e.scissor),this.scissorTest=e.scissorTest,this.viewport.copy(e.viewport),this.textures.length=0;for(let n=0,r=e.textures.length;n<r;n++)this.textures[n]=e.textures[n].clone(),this.textures[n].isRenderTargetTexture=!0;const t=Object.assign({},e.texture.image);return this.texture.source=new zc(t),this.depthBuffer=e.depthBuffer,this.stencilBuffer=e.stencilBuffer,this.resolveDepthBuffer=e.resolveDepthBuffer,this.resolveStencilBuffer=e.resolveStencilBuffer,e.depthTexture!==null&&(this.depthTexture=e.depthTexture.clone()),this.samples=e.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class Un extends Yd{constructor(e=1,t=1,n={}){super(e,t,n),this.isWebGLRenderTarget=!0}}class Vc extends St{constructor(e=null,t=1,n=1,r=1){super(null),this.isDataArrayTexture=!0,this.image={data:e,width:t,height:n,depth:r},this.magFilter=Pt,this.minFilter=Pt,this.wrapR=Pn,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(e){this.layerUpdates.add(e)}clearLayerUpdates(){this.layerUpdates.clear()}}class jd extends St{constructor(e=null,t=1,n=1,r=1){super(null),this.isData3DTexture=!0,this.image={data:e,width:t,height:n,depth:r},this.magFilter=Pt,this.minFilter=Pt,this.wrapR=Pn,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class Gt{constructor(e=0,t=0,n=0,r=1){this.isQuaternion=!0,this._x=e,this._y=t,this._z=n,this._w=r}static slerpFlat(e,t,n,r,s,o,a){let l=n[r+0],c=n[r+1],u=n[r+2],h=n[r+3];const d=s[o+0],f=s[o+1],g=s[o+2],x=s[o+3];if(a===0){e[t+0]=l,e[t+1]=c,e[t+2]=u,e[t+3]=h;return}if(a===1){e[t+0]=d,e[t+1]=f,e[t+2]=g,e[t+3]=x;return}if(h!==x||l!==d||c!==f||u!==g){let m=1-a;const p=l*d+c*f+u*g+h*x,w=p>=0?1:-1,S=1-p*p;if(S>Number.EPSILON){const k=Math.sqrt(S),I=Math.atan2(k,p*w);m=Math.sin(m*I)/k,a=Math.sin(a*I)/k}const A=a*w;if(l=l*m+d*A,c=c*m+f*A,u=u*m+g*A,h=h*m+x*A,m===1-a){const k=1/Math.sqrt(l*l+c*c+u*u+h*h);l*=k,c*=k,u*=k,h*=k}}e[t]=l,e[t+1]=c,e[t+2]=u,e[t+3]=h}static multiplyQuaternionsFlat(e,t,n,r,s,o){const a=n[r],l=n[r+1],c=n[r+2],u=n[r+3],h=s[o],d=s[o+1],f=s[o+2],g=s[o+3];return e[t]=a*g+u*h+l*f-c*d,e[t+1]=l*g+u*d+c*h-a*f,e[t+2]=c*g+u*f+a*d-l*h,e[t+3]=u*g-a*h-l*d-c*f,e}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get w(){return this._w}set w(e){this._w=e,this._onChangeCallback()}set(e,t,n,r){return this._x=e,this._y=t,this._z=n,this._w=r,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(e){return this._x=e.x,this._y=e.y,this._z=e.z,this._w=e.w,this._onChangeCallback(),this}setFromEuler(e,t=!0){const n=e._x,r=e._y,s=e._z,o=e._order,a=Math.cos,l=Math.sin,c=a(n/2),u=a(r/2),h=a(s/2),d=l(n/2),f=l(r/2),g=l(s/2);switch(o){case"XYZ":this._x=d*u*h+c*f*g,this._y=c*f*h-d*u*g,this._z=c*u*g+d*f*h,this._w=c*u*h-d*f*g;break;case"YXZ":this._x=d*u*h+c*f*g,this._y=c*f*h-d*u*g,this._z=c*u*g-d*f*h,this._w=c*u*h+d*f*g;break;case"ZXY":this._x=d*u*h-c*f*g,this._y=c*f*h+d*u*g,this._z=c*u*g+d*f*h,this._w=c*u*h-d*f*g;break;case"ZYX":this._x=d*u*h-c*f*g,this._y=c*f*h+d*u*g,this._z=c*u*g-d*f*h,this._w=c*u*h+d*f*g;break;case"YZX":this._x=d*u*h+c*f*g,this._y=c*f*h+d*u*g,this._z=c*u*g-d*f*h,this._w=c*u*h-d*f*g;break;case"XZY":this._x=d*u*h-c*f*g,this._y=c*f*h-d*u*g,this._z=c*u*g+d*f*h,this._w=c*u*h+d*f*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+o)}return t===!0&&this._onChangeCallback(),this}setFromAxisAngle(e,t){const n=t/2,r=Math.sin(n);return this._x=e.x*r,this._y=e.y*r,this._z=e.z*r,this._w=Math.cos(n),this._onChangeCallback(),this}setFromRotationMatrix(e){const t=e.elements,n=t[0],r=t[4],s=t[8],o=t[1],a=t[5],l=t[9],c=t[2],u=t[6],h=t[10],d=n+a+h;if(d>0){const f=.5/Math.sqrt(d+1);this._w=.25/f,this._x=(u-l)*f,this._y=(s-c)*f,this._z=(o-r)*f}else if(n>a&&n>h){const f=2*Math.sqrt(1+n-a-h);this._w=(u-l)/f,this._x=.25*f,this._y=(r+o)/f,this._z=(s+c)/f}else if(a>h){const f=2*Math.sqrt(1+a-n-h);this._w=(s-c)/f,this._x=(r+o)/f,this._y=.25*f,this._z=(l+u)/f}else{const f=2*Math.sqrt(1+h-n-a);this._w=(o-r)/f,this._x=(s+c)/f,this._y=(l+u)/f,this._z=.25*f}return this._onChangeCallback(),this}setFromUnitVectors(e,t){let n=e.dot(t)+1;return n<Number.EPSILON?(n=0,Math.abs(e.x)>Math.abs(e.z)?(this._x=-e.y,this._y=e.x,this._z=0,this._w=n):(this._x=0,this._y=-e.z,this._z=e.y,this._w=n)):(this._x=e.y*t.z-e.z*t.y,this._y=e.z*t.x-e.x*t.z,this._z=e.x*t.y-e.y*t.x,this._w=n),this.normalize()}angleTo(e){return 2*Math.acos(Math.abs(vt(this.dot(e),-1,1)))}rotateTowards(e,t){const n=this.angleTo(e);if(n===0)return this;const r=Math.min(1,t/n);return this.slerp(e,r),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(e){return this._x*e._x+this._y*e._y+this._z*e._z+this._w*e._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let e=this.length();return e===0?(this._x=0,this._y=0,this._z=0,this._w=1):(e=1/e,this._x=this._x*e,this._y=this._y*e,this._z=this._z*e,this._w=this._w*e),this._onChangeCallback(),this}multiply(e){return this.multiplyQuaternions(this,e)}premultiply(e){return this.multiplyQuaternions(e,this)}multiplyQuaternions(e,t){const n=e._x,r=e._y,s=e._z,o=e._w,a=t._x,l=t._y,c=t._z,u=t._w;return this._x=n*u+o*a+r*c-s*l,this._y=r*u+o*l+s*a-n*c,this._z=s*u+o*c+n*l-r*a,this._w=o*u-n*a-r*l-s*c,this._onChangeCallback(),this}slerp(e,t){if(t===0)return this;if(t===1)return this.copy(e);const n=this._x,r=this._y,s=this._z,o=this._w;let a=o*e._w+n*e._x+r*e._y+s*e._z;if(a<0?(this._w=-e._w,this._x=-e._x,this._y=-e._y,this._z=-e._z,a=-a):this.copy(e),a>=1)return this._w=o,this._x=n,this._y=r,this._z=s,this;const l=1-a*a;if(l<=Number.EPSILON){const f=1-t;return this._w=f*o+t*this._w,this._x=f*n+t*this._x,this._y=f*r+t*this._y,this._z=f*s+t*this._z,this.normalize(),this}const c=Math.sqrt(l),u=Math.atan2(c,a),h=Math.sin((1-t)*u)/c,d=Math.sin(t*u)/c;return this._w=o*h+this._w*d,this._x=n*h+this._x*d,this._y=r*h+this._y*d,this._z=s*h+this._z*d,this._onChangeCallback(),this}slerpQuaternions(e,t,n){return this.copy(e).slerp(t,n)}random(){const e=2*Math.PI*Math.random(),t=2*Math.PI*Math.random(),n=Math.random(),r=Math.sqrt(1-n),s=Math.sqrt(n);return this.set(r*Math.sin(e),r*Math.cos(e),s*Math.sin(t),s*Math.cos(t))}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._w===this._w}fromArray(e,t=0){return this._x=e[t],this._y=e[t+1],this._z=e[t+2],this._w=e[t+3],this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._w,e}fromBufferAttribute(e,t){return this._x=e.getX(t),this._y=e.getY(t),this._z=e.getZ(t),this._w=e.getW(t),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class G{constructor(e=0,t=0,n=0){G.prototype.isVector3=!0,this.x=e,this.y=t,this.z=n}set(e,t,n){return n===void 0&&(n=this.z),this.x=e,this.y=t,this.z=n,this}setScalar(e){return this.x=e,this.y=e,this.z=e,this}setX(e){return this.x=e,this}setY(e){return this.y=e,this}setZ(e){return this.z=e,this}setComponent(e,t){switch(e){case 0:this.x=t;break;case 1:this.y=t;break;case 2:this.z=t;break;default:throw new Error("index is out of range: "+e)}return this}getComponent(e){switch(e){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+e)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(e){return this.x=e.x,this.y=e.y,this.z=e.z,this}add(e){return this.x+=e.x,this.y+=e.y,this.z+=e.z,this}addScalar(e){return this.x+=e,this.y+=e,this.z+=e,this}addVectors(e,t){return this.x=e.x+t.x,this.y=e.y+t.y,this.z=e.z+t.z,this}addScaledVector(e,t){return this.x+=e.x*t,this.y+=e.y*t,this.z+=e.z*t,this}sub(e){return this.x-=e.x,this.y-=e.y,this.z-=e.z,this}subScalar(e){return this.x-=e,this.y-=e,this.z-=e,this}subVectors(e,t){return this.x=e.x-t.x,this.y=e.y-t.y,this.z=e.z-t.z,this}multiply(e){return this.x*=e.x,this.y*=e.y,this.z*=e.z,this}multiplyScalar(e){return this.x*=e,this.y*=e,this.z*=e,this}multiplyVectors(e,t){return this.x=e.x*t.x,this.y=e.y*t.y,this.z=e.z*t.z,this}applyEuler(e){return this.applyQuaternion(ll.setFromEuler(e))}applyAxisAngle(e,t){return this.applyQuaternion(ll.setFromAxisAngle(e,t))}applyMatrix3(e){const t=this.x,n=this.y,r=this.z,s=e.elements;return this.x=s[0]*t+s[3]*n+s[6]*r,this.y=s[1]*t+s[4]*n+s[7]*r,this.z=s[2]*t+s[5]*n+s[8]*r,this}applyNormalMatrix(e){return this.applyMatrix3(e).normalize()}applyMatrix4(e){const t=this.x,n=this.y,r=this.z,s=e.elements,o=1/(s[3]*t+s[7]*n+s[11]*r+s[15]);return this.x=(s[0]*t+s[4]*n+s[8]*r+s[12])*o,this.y=(s[1]*t+s[5]*n+s[9]*r+s[13])*o,this.z=(s[2]*t+s[6]*n+s[10]*r+s[14])*o,this}applyQuaternion(e){const t=this.x,n=this.y,r=this.z,s=e.x,o=e.y,a=e.z,l=e.w,c=2*(o*r-a*n),u=2*(a*t-s*r),h=2*(s*n-o*t);return this.x=t+l*c+o*h-a*u,this.y=n+l*u+a*c-s*h,this.z=r+l*h+s*u-o*c,this}project(e){return this.applyMatrix4(e.matrixWorldInverse).applyMatrix4(e.projectionMatrix)}unproject(e){return this.applyMatrix4(e.projectionMatrixInverse).applyMatrix4(e.matrixWorld)}transformDirection(e){const t=this.x,n=this.y,r=this.z,s=e.elements;return this.x=s[0]*t+s[4]*n+s[8]*r,this.y=s[1]*t+s[5]*n+s[9]*r,this.z=s[2]*t+s[6]*n+s[10]*r,this.normalize()}divide(e){return this.x/=e.x,this.y/=e.y,this.z/=e.z,this}divideScalar(e){return this.multiplyScalar(1/e)}min(e){return this.x=Math.min(this.x,e.x),this.y=Math.min(this.y,e.y),this.z=Math.min(this.z,e.z),this}max(e){return this.x=Math.max(this.x,e.x),this.y=Math.max(this.y,e.y),this.z=Math.max(this.z,e.z),this}clamp(e,t){return this.x=Math.max(e.x,Math.min(t.x,this.x)),this.y=Math.max(e.y,Math.min(t.y,this.y)),this.z=Math.max(e.z,Math.min(t.z,this.z)),this}clampScalar(e,t){return this.x=Math.max(e,Math.min(t,this.x)),this.y=Math.max(e,Math.min(t,this.y)),this.z=Math.max(e,Math.min(t,this.z)),this}clampLength(e,t){const n=this.length();return this.divideScalar(n||1).multiplyScalar(Math.max(e,Math.min(t,n)))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(e){return this.x*e.x+this.y*e.y+this.z*e.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(e){return this.normalize().multiplyScalar(e)}lerp(e,t){return this.x+=(e.x-this.x)*t,this.y+=(e.y-this.y)*t,this.z+=(e.z-this.z)*t,this}lerpVectors(e,t,n){return this.x=e.x+(t.x-e.x)*n,this.y=e.y+(t.y-e.y)*n,this.z=e.z+(t.z-e.z)*n,this}cross(e){return this.crossVectors(this,e)}crossVectors(e,t){const n=e.x,r=e.y,s=e.z,o=t.x,a=t.y,l=t.z;return this.x=r*l-s*a,this.y=s*o-n*l,this.z=n*a-r*o,this}projectOnVector(e){const t=e.lengthSq();if(t===0)return this.set(0,0,0);const n=e.dot(this)/t;return this.copy(e).multiplyScalar(n)}projectOnPlane(e){return Ds.copy(this).projectOnVector(e),this.sub(Ds)}reflect(e){return this.sub(Ds.copy(e).multiplyScalar(2*this.dot(e)))}angleTo(e){const t=Math.sqrt(this.lengthSq()*e.lengthSq());if(t===0)return Math.PI/2;const n=this.dot(e)/t;return Math.acos(vt(n,-1,1))}distanceTo(e){return Math.sqrt(this.distanceToSquared(e))}distanceToSquared(e){const t=this.x-e.x,n=this.y-e.y,r=this.z-e.z;return t*t+n*n+r*r}manhattanDistanceTo(e){return Math.abs(this.x-e.x)+Math.abs(this.y-e.y)+Math.abs(this.z-e.z)}setFromSpherical(e){return this.setFromSphericalCoords(e.radius,e.phi,e.theta)}setFromSphericalCoords(e,t,n){const r=Math.sin(t)*e;return this.x=r*Math.sin(n),this.y=Math.cos(t)*e,this.z=r*Math.cos(n),this}setFromCylindrical(e){return this.setFromCylindricalCoords(e.radius,e.theta,e.y)}setFromCylindricalCoords(e,t,n){return this.x=e*Math.sin(t),this.y=n,this.z=e*Math.cos(t),this}setFromMatrixPosition(e){const t=e.elements;return this.x=t[12],this.y=t[13],this.z=t[14],this}setFromMatrixScale(e){const t=this.setFromMatrixColumn(e,0).length(),n=this.setFromMatrixColumn(e,1).length(),r=this.setFromMatrixColumn(e,2).length();return this.x=t,this.y=n,this.z=r,this}setFromMatrixColumn(e,t){return this.fromArray(e.elements,t*4)}setFromMatrix3Column(e,t){return this.fromArray(e.elements,t*3)}setFromEuler(e){return this.x=e._x,this.y=e._y,this.z=e._z,this}setFromColor(e){return this.x=e.r,this.y=e.g,this.z=e.b,this}equals(e){return e.x===this.x&&e.y===this.y&&e.z===this.z}fromArray(e,t=0){return this.x=e[t],this.y=e[t+1],this.z=e[t+2],this}toArray(e=[],t=0){return e[t]=this.x,e[t+1]=this.y,e[t+2]=this.z,e}fromBufferAttribute(e,t){return this.x=e.getX(t),this.y=e.getY(t),this.z=e.getZ(t),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const e=Math.random()*Math.PI*2,t=Math.random()*2-1,n=Math.sqrt(1-t*t);return this.x=n*Math.cos(e),this.y=t,this.z=n*Math.sin(e),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const Ds=new G,ll=new Gt;class Li{constructor(e=new G(1/0,1/0,1/0),t=new G(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=e,this.max=t}set(e,t){return this.min.copy(e),this.max.copy(t),this}setFromArray(e){this.makeEmpty();for(let t=0,n=e.length;t<n;t+=3)this.expandByPoint(Ut.fromArray(e,t));return this}setFromBufferAttribute(e){this.makeEmpty();for(let t=0,n=e.count;t<n;t++)this.expandByPoint(Ut.fromBufferAttribute(e,t));return this}setFromPoints(e){this.makeEmpty();for(let t=0,n=e.length;t<n;t++)this.expandByPoint(e[t]);return this}setFromCenterAndSize(e,t){const n=Ut.copy(t).multiplyScalar(.5);return this.min.copy(e).sub(n),this.max.copy(e).add(n),this}setFromObject(e,t=!1){return this.makeEmpty(),this.expandByObject(e,t)}clone(){return new this.constructor().copy(this)}copy(e){return this.min.copy(e.min),this.max.copy(e.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(e){return this.isEmpty()?e.set(0,0,0):e.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(e){return this.isEmpty()?e.set(0,0,0):e.subVectors(this.max,this.min)}expandByPoint(e){return this.min.min(e),this.max.max(e),this}expandByVector(e){return this.min.sub(e),this.max.add(e),this}expandByScalar(e){return this.min.addScalar(-e),this.max.addScalar(e),this}expandByObject(e,t=!1){e.updateWorldMatrix(!1,!1);const n=e.geometry;if(n!==void 0){const s=n.getAttribute("position");if(t===!0&&s!==void 0&&e.isInstancedMesh!==!0)for(let o=0,a=s.count;o<a;o++)e.isMesh===!0?e.getVertexPosition(o,Ut):Ut.fromBufferAttribute(s,o),Ut.applyMatrix4(e.matrixWorld),this.expandByPoint(Ut);else e.boundingBox!==void 0?(e.boundingBox===null&&e.computeBoundingBox(),Fi.copy(e.boundingBox)):(n.boundingBox===null&&n.computeBoundingBox(),Fi.copy(n.boundingBox)),Fi.applyMatrix4(e.matrixWorld),this.union(Fi)}const r=e.children;for(let s=0,o=r.length;s<o;s++)this.expandByObject(r[s],t);return this}containsPoint(e){return!(e.x<this.min.x||e.x>this.max.x||e.y<this.min.y||e.y>this.max.y||e.z<this.min.z||e.z>this.max.z)}containsBox(e){return this.min.x<=e.min.x&&e.max.x<=this.max.x&&this.min.y<=e.min.y&&e.max.y<=this.max.y&&this.min.z<=e.min.z&&e.max.z<=this.max.z}getParameter(e,t){return t.set((e.x-this.min.x)/(this.max.x-this.min.x),(e.y-this.min.y)/(this.max.y-this.min.y),(e.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(e){return!(e.max.x<this.min.x||e.min.x>this.max.x||e.max.y<this.min.y||e.min.y>this.max.y||e.max.z<this.min.z||e.min.z>this.max.z)}intersectsSphere(e){return this.clampPoint(e.center,Ut),Ut.distanceToSquared(e.center)<=e.radius*e.radius}intersectsPlane(e){let t,n;return e.normal.x>0?(t=e.normal.x*this.min.x,n=e.normal.x*this.max.x):(t=e.normal.x*this.max.x,n=e.normal.x*this.min.x),e.normal.y>0?(t+=e.normal.y*this.min.y,n+=e.normal.y*this.max.y):(t+=e.normal.y*this.max.y,n+=e.normal.y*this.min.y),e.normal.z>0?(t+=e.normal.z*this.min.z,n+=e.normal.z*this.max.z):(t+=e.normal.z*this.max.z,n+=e.normal.z*this.min.z),t<=-e.constant&&n>=-e.constant}intersectsTriangle(e){if(this.isEmpty())return!1;this.getCenter(xi),Bi.subVectors(this.max,xi),Hn.subVectors(e.a,xi),Wn.subVectors(e.b,xi),Xn.subVectors(e.c,xi),tn.subVectors(Wn,Hn),nn.subVectors(Xn,Wn),Sn.subVectors(Hn,Xn);let t=[0,-tn.z,tn.y,0,-nn.z,nn.y,0,-Sn.z,Sn.y,tn.z,0,-tn.x,nn.z,0,-nn.x,Sn.z,0,-Sn.x,-tn.y,tn.x,0,-nn.y,nn.x,0,-Sn.y,Sn.x,0];return!Is(t,Hn,Wn,Xn,Bi)||(t=[1,0,0,0,1,0,0,0,1],!Is(t,Hn,Wn,Xn,Bi))?!1:(ki.crossVectors(tn,nn),t=[ki.x,ki.y,ki.z],Is(t,Hn,Wn,Xn,Bi))}clampPoint(e,t){return t.copy(e).clamp(this.min,this.max)}distanceToPoint(e){return this.clampPoint(e,Ut).distanceTo(e)}getBoundingSphere(e){return this.isEmpty()?e.makeEmpty():(this.getCenter(e.center),e.radius=this.getSize(Ut).length()*.5),e}intersect(e){return this.min.max(e.min),this.max.min(e.max),this.isEmpty()&&this.makeEmpty(),this}union(e){return this.min.min(e.min),this.max.max(e.max),this}applyMatrix4(e){return this.isEmpty()?this:(Xt[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(e),Xt[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(e),Xt[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(e),Xt[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(e),Xt[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(e),Xt[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(e),Xt[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(e),Xt[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(e),this.setFromPoints(Xt),this)}translate(e){return this.min.add(e),this.max.add(e),this}equals(e){return e.min.equals(this.min)&&e.max.equals(this.max)}}const Xt=[new G,new G,new G,new G,new G,new G,new G,new G],Ut=new G,Fi=new Li,Hn=new G,Wn=new G,Xn=new G,tn=new G,nn=new G,Sn=new G,xi=new G,Bi=new G,ki=new G,Mn=new G;function Is(i,e,t,n,r){for(let s=0,o=i.length-3;s<=o;s+=3){Mn.fromArray(i,s);const a=r.x*Math.abs(Mn.x)+r.y*Math.abs(Mn.y)+r.z*Math.abs(Mn.z),l=e.dot(Mn),c=t.dot(Mn),u=n.dot(Mn);if(Math.max(-Math.max(l,c,u),Math.min(l,c,u))>a)return!1}return!0}const $d=new Li,yi=new G,Ns=new G;class Ui{constructor(e=new G,t=-1){this.isSphere=!0,this.center=e,this.radius=t}set(e,t){return this.center.copy(e),this.radius=t,this}setFromPoints(e,t){const n=this.center;t!==void 0?n.copy(t):$d.setFromPoints(e).getCenter(n);let r=0;for(let s=0,o=e.length;s<o;s++)r=Math.max(r,n.distanceToSquared(e[s]));return this.radius=Math.sqrt(r),this}copy(e){return this.center.copy(e.center),this.radius=e.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(e){return e.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(e){return e.distanceTo(this.center)-this.radius}intersectsSphere(e){const t=this.radius+e.radius;return e.center.distanceToSquared(this.center)<=t*t}intersectsBox(e){return e.intersectsSphere(this)}intersectsPlane(e){return Math.abs(e.distanceToPoint(this.center))<=this.radius}clampPoint(e,t){const n=this.center.distanceToSquared(e);return t.copy(e),n>this.radius*this.radius&&(t.sub(this.center).normalize(),t.multiplyScalar(this.radius).add(this.center)),t}getBoundingBox(e){return this.isEmpty()?(e.makeEmpty(),e):(e.set(this.center,this.center),e.expandByScalar(this.radius),e)}applyMatrix4(e){return this.center.applyMatrix4(e),this.radius=this.radius*e.getMaxScaleOnAxis(),this}translate(e){return this.center.add(e),this}expandByPoint(e){if(this.isEmpty())return this.center.copy(e),this.radius=0,this;yi.subVectors(e,this.center);const t=yi.lengthSq();if(t>this.radius*this.radius){const n=Math.sqrt(t),r=(n-this.radius)*.5;this.center.addScaledVector(yi,r/n),this.radius+=r}return this}union(e){return e.isEmpty()?this:this.isEmpty()?(this.copy(e),this):(this.center.equals(e.center)===!0?this.radius=Math.max(this.radius,e.radius):(Ns.subVectors(e.center,this.center).setLength(e.radius),this.expandByPoint(yi.copy(e.center).add(Ns)),this.expandByPoint(yi.copy(e.center).sub(Ns))),this)}equals(e){return e.center.equals(this.center)&&e.radius===this.radius}clone(){return new this.constructor().copy(this)}}const qt=new G,Os=new G,zi=new G,rn=new G,Fs=new G,Vi=new G,Bs=new G;class wr{constructor(e=new G,t=new G(0,0,-1)){this.origin=e,this.direction=t}set(e,t){return this.origin.copy(e),this.direction.copy(t),this}copy(e){return this.origin.copy(e.origin),this.direction.copy(e.direction),this}at(e,t){return t.copy(this.origin).addScaledVector(this.direction,e)}lookAt(e){return this.direction.copy(e).sub(this.origin).normalize(),this}recast(e){return this.origin.copy(this.at(e,qt)),this}closestPointToPoint(e,t){t.subVectors(e,this.origin);const n=t.dot(this.direction);return n<0?t.copy(this.origin):t.copy(this.origin).addScaledVector(this.direction,n)}distanceToPoint(e){return Math.sqrt(this.distanceSqToPoint(e))}distanceSqToPoint(e){const t=qt.subVectors(e,this.origin).dot(this.direction);return t<0?this.origin.distanceToSquared(e):(qt.copy(this.origin).addScaledVector(this.direction,t),qt.distanceToSquared(e))}distanceSqToSegment(e,t,n,r){Os.copy(e).add(t).multiplyScalar(.5),zi.copy(t).sub(e).normalize(),rn.copy(this.origin).sub(Os);const s=e.distanceTo(t)*.5,o=-this.direction.dot(zi),a=rn.dot(this.direction),l=-rn.dot(zi),c=rn.lengthSq(),u=Math.abs(1-o*o);let h,d,f,g;if(u>0)if(h=o*l-a,d=o*a-l,g=s*u,h>=0)if(d>=-g)if(d<=g){const x=1/u;h*=x,d*=x,f=h*(h+o*d+2*a)+d*(o*h+d+2*l)+c}else d=s,h=Math.max(0,-(o*d+a)),f=-h*h+d*(d+2*l)+c;else d=-s,h=Math.max(0,-(o*d+a)),f=-h*h+d*(d+2*l)+c;else d<=-g?(h=Math.max(0,-(-o*s+a)),d=h>0?-s:Math.min(Math.max(-s,-l),s),f=-h*h+d*(d+2*l)+c):d<=g?(h=0,d=Math.min(Math.max(-s,-l),s),f=d*(d+2*l)+c):(h=Math.max(0,-(o*s+a)),d=h>0?s:Math.min(Math.max(-s,-l),s),f=-h*h+d*(d+2*l)+c);else d=o>0?-s:s,h=Math.max(0,-(o*d+a)),f=-h*h+d*(d+2*l)+c;return n&&n.copy(this.origin).addScaledVector(this.direction,h),r&&r.copy(Os).addScaledVector(zi,d),f}intersectSphere(e,t){qt.subVectors(e.center,this.origin);const n=qt.dot(this.direction),r=qt.dot(qt)-n*n,s=e.radius*e.radius;if(r>s)return null;const o=Math.sqrt(s-r),a=n-o,l=n+o;return l<0?null:a<0?this.at(l,t):this.at(a,t)}intersectsSphere(e){return this.distanceSqToPoint(e.center)<=e.radius*e.radius}distanceToPlane(e){const t=e.normal.dot(this.direction);if(t===0)return e.distanceToPoint(this.origin)===0?0:null;const n=-(this.origin.dot(e.normal)+e.constant)/t;return n>=0?n:null}intersectPlane(e,t){const n=this.distanceToPlane(e);return n===null?null:this.at(n,t)}intersectsPlane(e){const t=e.distanceToPoint(this.origin);return t===0||e.normal.dot(this.direction)*t<0}intersectBox(e,t){let n,r,s,o,a,l;const c=1/this.direction.x,u=1/this.direction.y,h=1/this.direction.z,d=this.origin;return c>=0?(n=(e.min.x-d.x)*c,r=(e.max.x-d.x)*c):(n=(e.max.x-d.x)*c,r=(e.min.x-d.x)*c),u>=0?(s=(e.min.y-d.y)*u,o=(e.max.y-d.y)*u):(s=(e.max.y-d.y)*u,o=(e.min.y-d.y)*u),n>o||s>r||((s>n||isNaN(n))&&(n=s),(o<r||isNaN(r))&&(r=o),h>=0?(a=(e.min.z-d.z)*h,l=(e.max.z-d.z)*h):(a=(e.max.z-d.z)*h,l=(e.min.z-d.z)*h),n>l||a>r)||((a>n||n!==n)&&(n=a),(l<r||r!==r)&&(r=l),r<0)?null:this.at(n>=0?n:r,t)}intersectsBox(e){return this.intersectBox(e,qt)!==null}intersectTriangle(e,t,n,r,s){Fs.subVectors(t,e),Vi.subVectors(n,e),Bs.crossVectors(Fs,Vi);let o=this.direction.dot(Bs),a;if(o>0){if(r)return null;a=1}else if(o<0)a=-1,o=-o;else return null;rn.subVectors(this.origin,e);const l=a*this.direction.dot(Vi.crossVectors(rn,Vi));if(l<0)return null;const c=a*this.direction.dot(Fs.cross(rn));if(c<0||l+c>o)return null;const u=-a*rn.dot(Bs);return u<0?null:this.at(u/o,s)}applyMatrix4(e){return this.origin.applyMatrix4(e),this.direction.transformDirection(e),this}equals(e){return e.origin.equals(this.origin)&&e.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class Je{constructor(e,t,n,r,s,o,a,l,c,u,h,d,f,g,x,m){Je.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],e!==void 0&&this.set(e,t,n,r,s,o,a,l,c,u,h,d,f,g,x,m)}set(e,t,n,r,s,o,a,l,c,u,h,d,f,g,x,m){const p=this.elements;return p[0]=e,p[4]=t,p[8]=n,p[12]=r,p[1]=s,p[5]=o,p[9]=a,p[13]=l,p[2]=c,p[6]=u,p[10]=h,p[14]=d,p[3]=f,p[7]=g,p[11]=x,p[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new Je().fromArray(this.elements)}copy(e){const t=this.elements,n=e.elements;return t[0]=n[0],t[1]=n[1],t[2]=n[2],t[3]=n[3],t[4]=n[4],t[5]=n[5],t[6]=n[6],t[7]=n[7],t[8]=n[8],t[9]=n[9],t[10]=n[10],t[11]=n[11],t[12]=n[12],t[13]=n[13],t[14]=n[14],t[15]=n[15],this}copyPosition(e){const t=this.elements,n=e.elements;return t[12]=n[12],t[13]=n[13],t[14]=n[14],this}setFromMatrix3(e){const t=e.elements;return this.set(t[0],t[3],t[6],0,t[1],t[4],t[7],0,t[2],t[5],t[8],0,0,0,0,1),this}extractBasis(e,t,n){return e.setFromMatrixColumn(this,0),t.setFromMatrixColumn(this,1),n.setFromMatrixColumn(this,2),this}makeBasis(e,t,n){return this.set(e.x,t.x,n.x,0,e.y,t.y,n.y,0,e.z,t.z,n.z,0,0,0,0,1),this}extractRotation(e){const t=this.elements,n=e.elements,r=1/qn.setFromMatrixColumn(e,0).length(),s=1/qn.setFromMatrixColumn(e,1).length(),o=1/qn.setFromMatrixColumn(e,2).length();return t[0]=n[0]*r,t[1]=n[1]*r,t[2]=n[2]*r,t[3]=0,t[4]=n[4]*s,t[5]=n[5]*s,t[6]=n[6]*s,t[7]=0,t[8]=n[8]*o,t[9]=n[9]*o,t[10]=n[10]*o,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromEuler(e){const t=this.elements,n=e.x,r=e.y,s=e.z,o=Math.cos(n),a=Math.sin(n),l=Math.cos(r),c=Math.sin(r),u=Math.cos(s),h=Math.sin(s);if(e.order==="XYZ"){const d=o*u,f=o*h,g=a*u,x=a*h;t[0]=l*u,t[4]=-l*h,t[8]=c,t[1]=f+g*c,t[5]=d-x*c,t[9]=-a*l,t[2]=x-d*c,t[6]=g+f*c,t[10]=o*l}else if(e.order==="YXZ"){const d=l*u,f=l*h,g=c*u,x=c*h;t[0]=d+x*a,t[4]=g*a-f,t[8]=o*c,t[1]=o*h,t[5]=o*u,t[9]=-a,t[2]=f*a-g,t[6]=x+d*a,t[10]=o*l}else if(e.order==="ZXY"){const d=l*u,f=l*h,g=c*u,x=c*h;t[0]=d-x*a,t[4]=-o*h,t[8]=g+f*a,t[1]=f+g*a,t[5]=o*u,t[9]=x-d*a,t[2]=-o*c,t[6]=a,t[10]=o*l}else if(e.order==="ZYX"){const d=o*u,f=o*h,g=a*u,x=a*h;t[0]=l*u,t[4]=g*c-f,t[8]=d*c+x,t[1]=l*h,t[5]=x*c+d,t[9]=f*c-g,t[2]=-c,t[6]=a*l,t[10]=o*l}else if(e.order==="YZX"){const d=o*l,f=o*c,g=a*l,x=a*c;t[0]=l*u,t[4]=x-d*h,t[8]=g*h+f,t[1]=h,t[5]=o*u,t[9]=-a*u,t[2]=-c*u,t[6]=f*h+g,t[10]=d-x*h}else if(e.order==="XZY"){const d=o*l,f=o*c,g=a*l,x=a*c;t[0]=l*u,t[4]=-h,t[8]=c*u,t[1]=d*h+x,t[5]=o*u,t[9]=f*h-g,t[2]=g*h-f,t[6]=a*u,t[10]=x*h+d}return t[3]=0,t[7]=0,t[11]=0,t[12]=0,t[13]=0,t[14]=0,t[15]=1,this}makeRotationFromQuaternion(e){return this.compose(Kd,e,Zd)}lookAt(e,t,n){const r=this.elements;return wt.subVectors(e,t),wt.lengthSq()===0&&(wt.z=1),wt.normalize(),sn.crossVectors(n,wt),sn.lengthSq()===0&&(Math.abs(n.z)===1?wt.x+=1e-4:wt.z+=1e-4,wt.normalize(),sn.crossVectors(n,wt)),sn.normalize(),Gi.crossVectors(wt,sn),r[0]=sn.x,r[4]=Gi.x,r[8]=wt.x,r[1]=sn.y,r[5]=Gi.y,r[9]=wt.y,r[2]=sn.z,r[6]=Gi.z,r[10]=wt.z,this}multiply(e){return this.multiplyMatrices(this,e)}premultiply(e){return this.multiplyMatrices(e,this)}multiplyMatrices(e,t){const n=e.elements,r=t.elements,s=this.elements,o=n[0],a=n[4],l=n[8],c=n[12],u=n[1],h=n[5],d=n[9],f=n[13],g=n[2],x=n[6],m=n[10],p=n[14],w=n[3],S=n[7],A=n[11],k=n[15],I=r[0],P=r[4],Y=r[8],E=r[12],b=r[1],C=r[5],H=r[9],O=r[13],$=r[2],z=r[6],q=r[10],te=r[14],v=r[3],T=r[7],U=r[11],N=r[15];return s[0]=o*I+a*b+l*$+c*v,s[4]=o*P+a*C+l*z+c*T,s[8]=o*Y+a*H+l*q+c*U,s[12]=o*E+a*O+l*te+c*N,s[1]=u*I+h*b+d*$+f*v,s[5]=u*P+h*C+d*z+f*T,s[9]=u*Y+h*H+d*q+f*U,s[13]=u*E+h*O+d*te+f*N,s[2]=g*I+x*b+m*$+p*v,s[6]=g*P+x*C+m*z+p*T,s[10]=g*Y+x*H+m*q+p*U,s[14]=g*E+x*O+m*te+p*N,s[3]=w*I+S*b+A*$+k*v,s[7]=w*P+S*C+A*z+k*T,s[11]=w*Y+S*H+A*q+k*U,s[15]=w*E+S*O+A*te+k*N,this}multiplyScalar(e){const t=this.elements;return t[0]*=e,t[4]*=e,t[8]*=e,t[12]*=e,t[1]*=e,t[5]*=e,t[9]*=e,t[13]*=e,t[2]*=e,t[6]*=e,t[10]*=e,t[14]*=e,t[3]*=e,t[7]*=e,t[11]*=e,t[15]*=e,this}determinant(){const e=this.elements,t=e[0],n=e[4],r=e[8],s=e[12],o=e[1],a=e[5],l=e[9],c=e[13],u=e[2],h=e[6],d=e[10],f=e[14],g=e[3],x=e[7],m=e[11],p=e[15];return g*(+s*l*h-r*c*h-s*a*d+n*c*d+r*a*f-n*l*f)+x*(+t*l*f-t*c*d+s*o*d-r*o*f+r*c*u-s*l*u)+m*(+t*c*h-t*a*f-s*o*h+n*o*f+s*a*u-n*c*u)+p*(-r*a*u-t*l*h+t*a*d+r*o*h-n*o*d+n*l*u)}transpose(){const e=this.elements;let t;return t=e[1],e[1]=e[4],e[4]=t,t=e[2],e[2]=e[8],e[8]=t,t=e[6],e[6]=e[9],e[9]=t,t=e[3],e[3]=e[12],e[12]=t,t=e[7],e[7]=e[13],e[13]=t,t=e[11],e[11]=e[14],e[14]=t,this}setPosition(e,t,n){const r=this.elements;return e.isVector3?(r[12]=e.x,r[13]=e.y,r[14]=e.z):(r[12]=e,r[13]=t,r[14]=n),this}invert(){const e=this.elements,t=e[0],n=e[1],r=e[2],s=e[3],o=e[4],a=e[5],l=e[6],c=e[7],u=e[8],h=e[9],d=e[10],f=e[11],g=e[12],x=e[13],m=e[14],p=e[15],w=h*m*c-x*d*c+x*l*f-a*m*f-h*l*p+a*d*p,S=g*d*c-u*m*c-g*l*f+o*m*f+u*l*p-o*d*p,A=u*x*c-g*h*c+g*a*f-o*x*f-u*a*p+o*h*p,k=g*h*l-u*x*l-g*a*d+o*x*d+u*a*m-o*h*m,I=t*w+n*S+r*A+s*k;if(I===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const P=1/I;return e[0]=w*P,e[1]=(x*d*s-h*m*s-x*r*f+n*m*f+h*r*p-n*d*p)*P,e[2]=(a*m*s-x*l*s+x*r*c-n*m*c-a*r*p+n*l*p)*P,e[3]=(h*l*s-a*d*s-h*r*c+n*d*c+a*r*f-n*l*f)*P,e[4]=S*P,e[5]=(u*m*s-g*d*s+g*r*f-t*m*f-u*r*p+t*d*p)*P,e[6]=(g*l*s-o*m*s-g*r*c+t*m*c+o*r*p-t*l*p)*P,e[7]=(o*d*s-u*l*s+u*r*c-t*d*c-o*r*f+t*l*f)*P,e[8]=A*P,e[9]=(g*h*s-u*x*s-g*n*f+t*x*f+u*n*p-t*h*p)*P,e[10]=(o*x*s-g*a*s+g*n*c-t*x*c-o*n*p+t*a*p)*P,e[11]=(u*a*s-o*h*s-u*n*c+t*h*c+o*n*f-t*a*f)*P,e[12]=k*P,e[13]=(u*x*r-g*h*r+g*n*d-t*x*d-u*n*m+t*h*m)*P,e[14]=(g*a*r-o*x*r-g*n*l+t*x*l+o*n*m-t*a*m)*P,e[15]=(o*h*r-u*a*r+u*n*l-t*h*l-o*n*d+t*a*d)*P,this}scale(e){const t=this.elements,n=e.x,r=e.y,s=e.z;return t[0]*=n,t[4]*=r,t[8]*=s,t[1]*=n,t[5]*=r,t[9]*=s,t[2]*=n,t[6]*=r,t[10]*=s,t[3]*=n,t[7]*=r,t[11]*=s,this}getMaxScaleOnAxis(){const e=this.elements,t=e[0]*e[0]+e[1]*e[1]+e[2]*e[2],n=e[4]*e[4]+e[5]*e[5]+e[6]*e[6],r=e[8]*e[8]+e[9]*e[9]+e[10]*e[10];return Math.sqrt(Math.max(t,n,r))}makeTranslation(e,t,n){return e.isVector3?this.set(1,0,0,e.x,0,1,0,e.y,0,0,1,e.z,0,0,0,1):this.set(1,0,0,e,0,1,0,t,0,0,1,n,0,0,0,1),this}makeRotationX(e){const t=Math.cos(e),n=Math.sin(e);return this.set(1,0,0,0,0,t,-n,0,0,n,t,0,0,0,0,1),this}makeRotationY(e){const t=Math.cos(e),n=Math.sin(e);return this.set(t,0,n,0,0,1,0,0,-n,0,t,0,0,0,0,1),this}makeRotationZ(e){const t=Math.cos(e),n=Math.sin(e);return this.set(t,-n,0,0,n,t,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(e,t){const n=Math.cos(t),r=Math.sin(t),s=1-n,o=e.x,a=e.y,l=e.z,c=s*o,u=s*a;return this.set(c*o+n,c*a-r*l,c*l+r*a,0,c*a+r*l,u*a+n,u*l-r*o,0,c*l-r*a,u*l+r*o,s*l*l+n,0,0,0,0,1),this}makeScale(e,t,n){return this.set(e,0,0,0,0,t,0,0,0,0,n,0,0,0,0,1),this}makeShear(e,t,n,r,s,o){return this.set(1,n,s,0,e,1,o,0,t,r,1,0,0,0,0,1),this}compose(e,t,n){const r=this.elements,s=t._x,o=t._y,a=t._z,l=t._w,c=s+s,u=o+o,h=a+a,d=s*c,f=s*u,g=s*h,x=o*u,m=o*h,p=a*h,w=l*c,S=l*u,A=l*h,k=n.x,I=n.y,P=n.z;return r[0]=(1-(x+p))*k,r[1]=(f+A)*k,r[2]=(g-S)*k,r[3]=0,r[4]=(f-A)*I,r[5]=(1-(d+p))*I,r[6]=(m+w)*I,r[7]=0,r[8]=(g+S)*P,r[9]=(m-w)*P,r[10]=(1-(d+x))*P,r[11]=0,r[12]=e.x,r[13]=e.y,r[14]=e.z,r[15]=1,this}decompose(e,t,n){const r=this.elements;let s=qn.set(r[0],r[1],r[2]).length();const o=qn.set(r[4],r[5],r[6]).length(),a=qn.set(r[8],r[9],r[10]).length();this.determinant()<0&&(s=-s),e.x=r[12],e.y=r[13],e.z=r[14],Dt.copy(this);const c=1/s,u=1/o,h=1/a;return Dt.elements[0]*=c,Dt.elements[1]*=c,Dt.elements[2]*=c,Dt.elements[4]*=u,Dt.elements[5]*=u,Dt.elements[6]*=u,Dt.elements[8]*=h,Dt.elements[9]*=h,Dt.elements[10]*=h,t.setFromRotationMatrix(Dt),n.x=s,n.y=o,n.z=a,this}makePerspective(e,t,n,r,s,o,a=Jt){const l=this.elements,c=2*s/(t-e),u=2*s/(n-r),h=(t+e)/(t-e),d=(n+r)/(n-r);let f,g;if(a===Jt)f=-(o+s)/(o-s),g=-2*o*s/(o-s);else if(a===_r)f=-o/(o-s),g=-o*s/(o-s);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+a);return l[0]=c,l[4]=0,l[8]=h,l[12]=0,l[1]=0,l[5]=u,l[9]=d,l[13]=0,l[2]=0,l[6]=0,l[10]=f,l[14]=g,l[3]=0,l[7]=0,l[11]=-1,l[15]=0,this}makeOrthographic(e,t,n,r,s,o,a=Jt){const l=this.elements,c=1/(t-e),u=1/(n-r),h=1/(o-s),d=(t+e)*c,f=(n+r)*u;let g,x;if(a===Jt)g=(o+s)*h,x=-2*h;else if(a===_r)g=s*h,x=-1*h;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+a);return l[0]=2*c,l[4]=0,l[8]=0,l[12]=-d,l[1]=0,l[5]=2*u,l[9]=0,l[13]=-f,l[2]=0,l[6]=0,l[10]=x,l[14]=-g,l[3]=0,l[7]=0,l[11]=0,l[15]=1,this}equals(e){const t=this.elements,n=e.elements;for(let r=0;r<16;r++)if(t[r]!==n[r])return!1;return!0}fromArray(e,t=0){for(let n=0;n<16;n++)this.elements[n]=e[n+t];return this}toArray(e=[],t=0){const n=this.elements;return e[t]=n[0],e[t+1]=n[1],e[t+2]=n[2],e[t+3]=n[3],e[t+4]=n[4],e[t+5]=n[5],e[t+6]=n[6],e[t+7]=n[7],e[t+8]=n[8],e[t+9]=n[9],e[t+10]=n[10],e[t+11]=n[11],e[t+12]=n[12],e[t+13]=n[13],e[t+14]=n[14],e[t+15]=n[15],e}}const qn=new G,Dt=new Je,Kd=new G(0,0,0),Zd=new G(1,1,1),sn=new G,Gi=new G,wt=new G,cl=new Je,ul=new Gt;class Ht{constructor(e=0,t=0,n=0,r=Ht.DEFAULT_ORDER){this.isEuler=!0,this._x=e,this._y=t,this._z=n,this._order=r}get x(){return this._x}set x(e){this._x=e,this._onChangeCallback()}get y(){return this._y}set y(e){this._y=e,this._onChangeCallback()}get z(){return this._z}set z(e){this._z=e,this._onChangeCallback()}get order(){return this._order}set order(e){this._order=e,this._onChangeCallback()}set(e,t,n,r=this._order){return this._x=e,this._y=t,this._z=n,this._order=r,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(e){return this._x=e._x,this._y=e._y,this._z=e._z,this._order=e._order,this._onChangeCallback(),this}setFromRotationMatrix(e,t=this._order,n=!0){const r=e.elements,s=r[0],o=r[4],a=r[8],l=r[1],c=r[5],u=r[9],h=r[2],d=r[6],f=r[10];switch(t){case"XYZ":this._y=Math.asin(vt(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(-u,f),this._z=Math.atan2(-o,s)):(this._x=Math.atan2(d,c),this._z=0);break;case"YXZ":this._x=Math.asin(-vt(u,-1,1)),Math.abs(u)<.9999999?(this._y=Math.atan2(a,f),this._z=Math.atan2(l,c)):(this._y=Math.atan2(-h,s),this._z=0);break;case"ZXY":this._x=Math.asin(vt(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-h,f),this._z=Math.atan2(-o,c)):(this._y=0,this._z=Math.atan2(l,s));break;case"ZYX":this._y=Math.asin(-vt(h,-1,1)),Math.abs(h)<.9999999?(this._x=Math.atan2(d,f),this._z=Math.atan2(l,s)):(this._x=0,this._z=Math.atan2(-o,c));break;case"YZX":this._z=Math.asin(vt(l,-1,1)),Math.abs(l)<.9999999?(this._x=Math.atan2(-u,c),this._y=Math.atan2(-h,s)):(this._x=0,this._y=Math.atan2(a,f));break;case"XZY":this._z=Math.asin(-vt(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(d,c),this._y=Math.atan2(a,s)):(this._x=Math.atan2(-u,f),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+t)}return this._order=t,n===!0&&this._onChangeCallback(),this}setFromQuaternion(e,t,n){return cl.makeRotationFromQuaternion(e),this.setFromRotationMatrix(cl,t,n)}setFromVector3(e,t=this._order){return this.set(e.x,e.y,e.z,t)}reorder(e){return ul.setFromEuler(this),this.setFromQuaternion(ul,e)}equals(e){return e._x===this._x&&e._y===this._y&&e._z===this._z&&e._order===this._order}fromArray(e){return this._x=e[0],this._y=e[1],this._z=e[2],e[3]!==void 0&&(this._order=e[3]),this._onChangeCallback(),this}toArray(e=[],t=0){return e[t]=this._x,e[t+1]=this._y,e[t+2]=this._z,e[t+3]=this._order,e}_onChange(e){return this._onChangeCallback=e,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}Ht.DEFAULT_ORDER="XYZ";class Gc{constructor(){this.mask=1}set(e){this.mask=(1<<e|0)>>>0}enable(e){this.mask|=1<<e|0}enableAll(){this.mask=-1}toggle(e){this.mask^=1<<e|0}disable(e){this.mask&=~(1<<e|0)}disableAll(){this.mask=0}test(e){return(this.mask&e.mask)!==0}isEnabled(e){return(this.mask&(1<<e|0))!==0}}let Jd=0;const hl=new G,Yn=new Gt,Yt=new Je,Hi=new G,Si=new G,Qd=new G,ef=new Gt,dl=new G(1,0,0),fl=new G(0,1,0),pl=new G(0,0,1),ml={type:"added"},tf={type:"removed"},jn={type:"childadded",child:null},ks={type:"childremoved",child:null};class dt extends Fn{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:Jd++}),this.uuid=Pi(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=dt.DEFAULT_UP.clone();const e=new G,t=new Ht,n=new Gt,r=new G(1,1,1);function s(){n.setFromEuler(t,!1)}function o(){t.setFromQuaternion(n,void 0,!1)}t._onChange(s),n._onChange(o),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:e},rotation:{configurable:!0,enumerable:!0,value:t},quaternion:{configurable:!0,enumerable:!0,value:n},scale:{configurable:!0,enumerable:!0,value:r},modelViewMatrix:{value:new Je},normalMatrix:{value:new ze}}),this.matrix=new Je,this.matrixWorld=new Je,this.matrixAutoUpdate=dt.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=dt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new Gc,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(e){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(e),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(e){return this.quaternion.premultiply(e),this}setRotationFromAxisAngle(e,t){this.quaternion.setFromAxisAngle(e,t)}setRotationFromEuler(e){this.quaternion.setFromEuler(e,!0)}setRotationFromMatrix(e){this.quaternion.setFromRotationMatrix(e)}setRotationFromQuaternion(e){this.quaternion.copy(e)}rotateOnAxis(e,t){return Yn.setFromAxisAngle(e,t),this.quaternion.multiply(Yn),this}rotateOnWorldAxis(e,t){return Yn.setFromAxisAngle(e,t),this.quaternion.premultiply(Yn),this}rotateX(e){return this.rotateOnAxis(dl,e)}rotateY(e){return this.rotateOnAxis(fl,e)}rotateZ(e){return this.rotateOnAxis(pl,e)}translateOnAxis(e,t){return hl.copy(e).applyQuaternion(this.quaternion),this.position.add(hl.multiplyScalar(t)),this}translateX(e){return this.translateOnAxis(dl,e)}translateY(e){return this.translateOnAxis(fl,e)}translateZ(e){return this.translateOnAxis(pl,e)}localToWorld(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(this.matrixWorld)}worldToLocal(e){return this.updateWorldMatrix(!0,!1),e.applyMatrix4(Yt.copy(this.matrixWorld).invert())}lookAt(e,t,n){e.isVector3?Hi.copy(e):Hi.set(e,t,n);const r=this.parent;this.updateWorldMatrix(!0,!1),Si.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?Yt.lookAt(Si,Hi,this.up):Yt.lookAt(Hi,Si,this.up),this.quaternion.setFromRotationMatrix(Yt),r&&(Yt.extractRotation(r.matrixWorld),Yn.setFromRotationMatrix(Yt),this.quaternion.premultiply(Yn.invert()))}add(e){if(arguments.length>1){for(let t=0;t<arguments.length;t++)this.add(arguments[t]);return this}return e===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",e),this):(e&&e.isObject3D?(e.removeFromParent(),e.parent=this,this.children.push(e),e.dispatchEvent(ml),jn.child=e,this.dispatchEvent(jn),jn.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",e),this)}remove(e){if(arguments.length>1){for(let n=0;n<arguments.length;n++)this.remove(arguments[n]);return this}const t=this.children.indexOf(e);return t!==-1&&(e.parent=null,this.children.splice(t,1),e.dispatchEvent(tf),ks.child=e,this.dispatchEvent(ks),ks.child=null),this}removeFromParent(){const e=this.parent;return e!==null&&e.remove(this),this}clear(){return this.remove(...this.children)}attach(e){return this.updateWorldMatrix(!0,!1),Yt.copy(this.matrixWorld).invert(),e.parent!==null&&(e.parent.updateWorldMatrix(!0,!1),Yt.multiply(e.parent.matrixWorld)),e.applyMatrix4(Yt),e.removeFromParent(),e.parent=this,this.children.push(e),e.updateWorldMatrix(!1,!0),e.dispatchEvent(ml),jn.child=e,this.dispatchEvent(jn),jn.child=null,this}getObjectById(e){return this.getObjectByProperty("id",e)}getObjectByName(e){return this.getObjectByProperty("name",e)}getObjectByProperty(e,t){if(this[e]===t)return this;for(let n=0,r=this.children.length;n<r;n++){const o=this.children[n].getObjectByProperty(e,t);if(o!==void 0)return o}}getObjectsByProperty(e,t,n=[]){this[e]===t&&n.push(this);const r=this.children;for(let s=0,o=r.length;s<o;s++)r[s].getObjectsByProperty(e,t,n);return n}getWorldPosition(e){return this.updateWorldMatrix(!0,!1),e.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Si,e,Qd),e}getWorldScale(e){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(Si,ef,e),e}getWorldDirection(e){this.updateWorldMatrix(!0,!1);const t=this.matrixWorld.elements;return e.set(t[8],t[9],t[10]).normalize()}raycast(){}traverse(e){e(this);const t=this.children;for(let n=0,r=t.length;n<r;n++)t[n].traverse(e)}traverseVisible(e){if(this.visible===!1)return;e(this);const t=this.children;for(let n=0,r=t.length;n<r;n++)t[n].traverseVisible(e)}traverseAncestors(e){const t=this.parent;t!==null&&(e(t),t.traverseAncestors(e))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(e){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||e)&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),this.matrixWorldNeedsUpdate=!1,e=!0);const t=this.children;for(let n=0,r=t.length;n<r;n++){const s=t[n];(s.matrixWorldAutoUpdate===!0||e===!0)&&s.updateMatrixWorld(e)}}updateWorldMatrix(e,t){const n=this.parent;if(e===!0&&n!==null&&n.matrixWorldAutoUpdate===!0&&n.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix),t===!0){const r=this.children;for(let s=0,o=r.length;s<o;s++){const a=r[s];a.matrixWorldAutoUpdate===!0&&a.updateWorldMatrix(!1,!0)}}}toJSON(e){const t=e===void 0||typeof e=="string",n={};t&&(e={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},n.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const r={};r.uuid=this.uuid,r.type=this.type,this.name!==""&&(r.name=this.name),this.castShadow===!0&&(r.castShadow=!0),this.receiveShadow===!0&&(r.receiveShadow=!0),this.visible===!1&&(r.visible=!1),this.frustumCulled===!1&&(r.frustumCulled=!1),this.renderOrder!==0&&(r.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(r.userData=this.userData),r.layers=this.layers.mask,r.matrix=this.matrix.toArray(),r.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(r.matrixAutoUpdate=!1),this.isInstancedMesh&&(r.type="InstancedMesh",r.count=this.count,r.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(r.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(r.type="BatchedMesh",r.perObjectFrustumCulled=this.perObjectFrustumCulled,r.sortObjects=this.sortObjects,r.drawRanges=this._drawRanges,r.reservedRanges=this._reservedRanges,r.visibility=this._visibility,r.active=this._active,r.bounds=this._bounds.map(a=>({boxInitialized:a.boxInitialized,boxMin:a.box.min.toArray(),boxMax:a.box.max.toArray(),sphereInitialized:a.sphereInitialized,sphereRadius:a.sphere.radius,sphereCenter:a.sphere.center.toArray()})),r.maxGeometryCount=this._maxGeometryCount,r.maxVertexCount=this._maxVertexCount,r.maxIndexCount=this._maxIndexCount,r.geometryInitialized=this._geometryInitialized,r.geometryCount=this._geometryCount,r.matricesTexture=this._matricesTexture.toJSON(e),this._colorsTexture!==null&&(r.colorsTexture=this._colorsTexture.toJSON(e)),this.boundingSphere!==null&&(r.boundingSphere={center:r.boundingSphere.center.toArray(),radius:r.boundingSphere.radius}),this.boundingBox!==null&&(r.boundingBox={min:r.boundingBox.min.toArray(),max:r.boundingBox.max.toArray()}));function s(a,l){return a[l.uuid]===void 0&&(a[l.uuid]=l.toJSON(e)),l.uuid}if(this.isScene)this.background&&(this.background.isColor?r.background=this.background.toJSON():this.background.isTexture&&(r.background=this.background.toJSON(e).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(r.environment=this.environment.toJSON(e).uuid);else if(this.isMesh||this.isLine||this.isPoints){r.geometry=s(e.geometries,this.geometry);const a=this.geometry.parameters;if(a!==void 0&&a.shapes!==void 0){const l=a.shapes;if(Array.isArray(l))for(let c=0,u=l.length;c<u;c++){const h=l[c];s(e.shapes,h)}else s(e.shapes,l)}}if(this.isSkinnedMesh&&(r.bindMode=this.bindMode,r.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(s(e.skeletons,this.skeleton),r.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const a=[];for(let l=0,c=this.material.length;l<c;l++)a.push(s(e.materials,this.material[l]));r.material=a}else r.material=s(e.materials,this.material);if(this.children.length>0){r.children=[];for(let a=0;a<this.children.length;a++)r.children.push(this.children[a].toJSON(e).object)}if(this.animations.length>0){r.animations=[];for(let a=0;a<this.animations.length;a++){const l=this.animations[a];r.animations.push(s(e.animations,l))}}if(t){const a=o(e.geometries),l=o(e.materials),c=o(e.textures),u=o(e.images),h=o(e.shapes),d=o(e.skeletons),f=o(e.animations),g=o(e.nodes);a.length>0&&(n.geometries=a),l.length>0&&(n.materials=l),c.length>0&&(n.textures=c),u.length>0&&(n.images=u),h.length>0&&(n.shapes=h),d.length>0&&(n.skeletons=d),f.length>0&&(n.animations=f),g.length>0&&(n.nodes=g)}return n.object=r,n;function o(a){const l=[];for(const c in a){const u=a[c];delete u.metadata,l.push(u)}return l}}clone(e){return new this.constructor().copy(this,e)}copy(e,t=!0){if(this.name=e.name,this.up.copy(e.up),this.position.copy(e.position),this.rotation.order=e.rotation.order,this.quaternion.copy(e.quaternion),this.scale.copy(e.scale),this.matrix.copy(e.matrix),this.matrixWorld.copy(e.matrixWorld),this.matrixAutoUpdate=e.matrixAutoUpdate,this.matrixWorldAutoUpdate=e.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=e.matrixWorldNeedsUpdate,this.layers.mask=e.layers.mask,this.visible=e.visible,this.castShadow=e.castShadow,this.receiveShadow=e.receiveShadow,this.frustumCulled=e.frustumCulled,this.renderOrder=e.renderOrder,this.animations=e.animations.slice(),this.userData=JSON.parse(JSON.stringify(e.userData)),t===!0)for(let n=0;n<e.children.length;n++){const r=e.children[n];this.add(r.clone())}return this}}dt.DEFAULT_UP=new G(0,1,0);dt.DEFAULT_MATRIX_AUTO_UPDATE=!0;dt.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const It=new G,jt=new G,zs=new G,$t=new G,$n=new G,Kn=new G,gl=new G,Vs=new G,Gs=new G,Hs=new G;class kt{constructor(e=new G,t=new G,n=new G){this.a=e,this.b=t,this.c=n}static getNormal(e,t,n,r){r.subVectors(n,t),It.subVectors(e,t),r.cross(It);const s=r.lengthSq();return s>0?r.multiplyScalar(1/Math.sqrt(s)):r.set(0,0,0)}static getBarycoord(e,t,n,r,s){It.subVectors(r,t),jt.subVectors(n,t),zs.subVectors(e,t);const o=It.dot(It),a=It.dot(jt),l=It.dot(zs),c=jt.dot(jt),u=jt.dot(zs),h=o*c-a*a;if(h===0)return s.set(0,0,0),null;const d=1/h,f=(c*l-a*u)*d,g=(o*u-a*l)*d;return s.set(1-f-g,g,f)}static containsPoint(e,t,n,r){return this.getBarycoord(e,t,n,r,$t)===null?!1:$t.x>=0&&$t.y>=0&&$t.x+$t.y<=1}static getInterpolation(e,t,n,r,s,o,a,l){return this.getBarycoord(e,t,n,r,$t)===null?(l.x=0,l.y=0,"z"in l&&(l.z=0),"w"in l&&(l.w=0),null):(l.setScalar(0),l.addScaledVector(s,$t.x),l.addScaledVector(o,$t.y),l.addScaledVector(a,$t.z),l)}static isFrontFacing(e,t,n,r){return It.subVectors(n,t),jt.subVectors(e,t),It.cross(jt).dot(r)<0}set(e,t,n){return this.a.copy(e),this.b.copy(t),this.c.copy(n),this}setFromPointsAndIndices(e,t,n,r){return this.a.copy(e[t]),this.b.copy(e[n]),this.c.copy(e[r]),this}setFromAttributeAndIndices(e,t,n,r){return this.a.fromBufferAttribute(e,t),this.b.fromBufferAttribute(e,n),this.c.fromBufferAttribute(e,r),this}clone(){return new this.constructor().copy(this)}copy(e){return this.a.copy(e.a),this.b.copy(e.b),this.c.copy(e.c),this}getArea(){return It.subVectors(this.c,this.b),jt.subVectors(this.a,this.b),It.cross(jt).length()*.5}getMidpoint(e){return e.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(e){return kt.getNormal(this.a,this.b,this.c,e)}getPlane(e){return e.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(e,t){return kt.getBarycoord(e,this.a,this.b,this.c,t)}getInterpolation(e,t,n,r,s){return kt.getInterpolation(e,this.a,this.b,this.c,t,n,r,s)}containsPoint(e){return kt.containsPoint(e,this.a,this.b,this.c)}isFrontFacing(e){return kt.isFrontFacing(this.a,this.b,this.c,e)}intersectsBox(e){return e.intersectsTriangle(this)}closestPointToPoint(e,t){const n=this.a,r=this.b,s=this.c;let o,a;$n.subVectors(r,n),Kn.subVectors(s,n),Vs.subVectors(e,n);const l=$n.dot(Vs),c=Kn.dot(Vs);if(l<=0&&c<=0)return t.copy(n);Gs.subVectors(e,r);const u=$n.dot(Gs),h=Kn.dot(Gs);if(u>=0&&h<=u)return t.copy(r);const d=l*h-u*c;if(d<=0&&l>=0&&u<=0)return o=l/(l-u),t.copy(n).addScaledVector($n,o);Hs.subVectors(e,s);const f=$n.dot(Hs),g=Kn.dot(Hs);if(g>=0&&f<=g)return t.copy(s);const x=f*c-l*g;if(x<=0&&c>=0&&g<=0)return a=c/(c-g),t.copy(n).addScaledVector(Kn,a);const m=u*g-f*h;if(m<=0&&h-u>=0&&f-g>=0)return gl.subVectors(s,r),a=(h-u)/(h-u+(f-g)),t.copy(r).addScaledVector(gl,a);const p=1/(m+x+d);return o=x*p,a=d*p,t.copy(n).addScaledVector($n,o).addScaledVector(Kn,a)}equals(e){return e.a.equals(this.a)&&e.b.equals(this.b)&&e.c.equals(this.c)}}const Hc={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},an={h:0,s:0,l:0},Wi={h:0,s:0,l:0};function Ws(i,e,t){return t<0&&(t+=1),t>1&&(t-=1),t<1/6?i+(e-i)*6*t:t<1/2?e:t<2/3?i+(e-i)*6*(2/3-t):i}class He{constructor(e,t,n){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(e,t,n)}set(e,t,n){if(t===void 0&&n===void 0){const r=e;r&&r.isColor?this.copy(r):typeof r=="number"?this.setHex(r):typeof r=="string"&&this.setStyle(r)}else this.setRGB(e,t,n);return this}setScalar(e){return this.r=e,this.g=e,this.b=e,this}setHex(e,t=Ft){return e=Math.floor(e),this.r=(e>>16&255)/255,this.g=(e>>8&255)/255,this.b=(e&255)/255,je.toWorkingColorSpace(this,t),this}setRGB(e,t,n,r=je.workingColorSpace){return this.r=e,this.g=t,this.b=n,je.toWorkingColorSpace(this,r),this}setHSL(e,t,n,r=je.workingColorSpace){if(e=kd(e,1),t=vt(t,0,1),n=vt(n,0,1),t===0)this.r=this.g=this.b=n;else{const s=n<=.5?n*(1+t):n+t-n*t,o=2*n-s;this.r=Ws(o,s,e+1/3),this.g=Ws(o,s,e),this.b=Ws(o,s,e-1/3)}return je.toWorkingColorSpace(this,r),this}setStyle(e,t=Ft){function n(s){s!==void 0&&parseFloat(s)<1&&console.warn("THREE.Color: Alpha component of "+e+" will be ignored.")}let r;if(r=/^(\w+)\(([^\)]*)\)/.exec(e)){let s;const o=r[1],a=r[2];switch(o){case"rgb":case"rgba":if(s=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(s[4]),this.setRGB(Math.min(255,parseInt(s[1],10))/255,Math.min(255,parseInt(s[2],10))/255,Math.min(255,parseInt(s[3],10))/255,t);if(s=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(s[4]),this.setRGB(Math.min(100,parseInt(s[1],10))/100,Math.min(100,parseInt(s[2],10))/100,Math.min(100,parseInt(s[3],10))/100,t);break;case"hsl":case"hsla":if(s=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(a))return n(s[4]),this.setHSL(parseFloat(s[1])/360,parseFloat(s[2])/100,parseFloat(s[3])/100,t);break;default:console.warn("THREE.Color: Unknown color model "+e)}}else if(r=/^\#([A-Fa-f\d]+)$/.exec(e)){const s=r[1],o=s.length;if(o===3)return this.setRGB(parseInt(s.charAt(0),16)/15,parseInt(s.charAt(1),16)/15,parseInt(s.charAt(2),16)/15,t);if(o===6)return this.setHex(parseInt(s,16),t);console.warn("THREE.Color: Invalid hex color "+e)}else if(e&&e.length>0)return this.setColorName(e,t);return this}setColorName(e,t=Ft){const n=Hc[e.toLowerCase()];return n!==void 0?this.setHex(n,t):console.warn("THREE.Color: Unknown color "+e),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(e){return this.r=e.r,this.g=e.g,this.b=e.b,this}copySRGBToLinear(e){return this.r=oi(e.r),this.g=oi(e.g),this.b=oi(e.b),this}copyLinearToSRGB(e){return this.r=Ls(e.r),this.g=Ls(e.g),this.b=Ls(e.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(e=Ft){return je.fromWorkingColorSpace(mt.copy(this),e),Math.round(vt(mt.r*255,0,255))*65536+Math.round(vt(mt.g*255,0,255))*256+Math.round(vt(mt.b*255,0,255))}getHexString(e=Ft){return("000000"+this.getHex(e).toString(16)).slice(-6)}getHSL(e,t=je.workingColorSpace){je.fromWorkingColorSpace(mt.copy(this),t);const n=mt.r,r=mt.g,s=mt.b,o=Math.max(n,r,s),a=Math.min(n,r,s);let l,c;const u=(a+o)/2;if(a===o)l=0,c=0;else{const h=o-a;switch(c=u<=.5?h/(o+a):h/(2-o-a),o){case n:l=(r-s)/h+(r<s?6:0);break;case r:l=(s-n)/h+2;break;case s:l=(n-r)/h+4;break}l/=6}return e.h=l,e.s=c,e.l=u,e}getRGB(e,t=je.workingColorSpace){return je.fromWorkingColorSpace(mt.copy(this),t),e.r=mt.r,e.g=mt.g,e.b=mt.b,e}getStyle(e=Ft){je.fromWorkingColorSpace(mt.copy(this),e);const t=mt.r,n=mt.g,r=mt.b;return e!==Ft?`color(${e} ${t.toFixed(3)} ${n.toFixed(3)} ${r.toFixed(3)})`:`rgb(${Math.round(t*255)},${Math.round(n*255)},${Math.round(r*255)})`}offsetHSL(e,t,n){return this.getHSL(an),this.setHSL(an.h+e,an.s+t,an.l+n)}add(e){return this.r+=e.r,this.g+=e.g,this.b+=e.b,this}addColors(e,t){return this.r=e.r+t.r,this.g=e.g+t.g,this.b=e.b+t.b,this}addScalar(e){return this.r+=e,this.g+=e,this.b+=e,this}sub(e){return this.r=Math.max(0,this.r-e.r),this.g=Math.max(0,this.g-e.g),this.b=Math.max(0,this.b-e.b),this}multiply(e){return this.r*=e.r,this.g*=e.g,this.b*=e.b,this}multiplyScalar(e){return this.r*=e,this.g*=e,this.b*=e,this}lerp(e,t){return this.r+=(e.r-this.r)*t,this.g+=(e.g-this.g)*t,this.b+=(e.b-this.b)*t,this}lerpColors(e,t,n){return this.r=e.r+(t.r-e.r)*n,this.g=e.g+(t.g-e.g)*n,this.b=e.b+(t.b-e.b)*n,this}lerpHSL(e,t){this.getHSL(an),e.getHSL(Wi);const n=Rs(an.h,Wi.h,t),r=Rs(an.s,Wi.s,t),s=Rs(an.l,Wi.l,t);return this.setHSL(n,r,s),this}setFromVector3(e){return this.r=e.x,this.g=e.y,this.b=e.z,this}applyMatrix3(e){const t=this.r,n=this.g,r=this.b,s=e.elements;return this.r=s[0]*t+s[3]*n+s[6]*r,this.g=s[1]*t+s[4]*n+s[7]*r,this.b=s[2]*t+s[5]*n+s[8]*r,this}equals(e){return e.r===this.r&&e.g===this.g&&e.b===this.b}fromArray(e,t=0){return this.r=e[t],this.g=e[t+1],this.b=e[t+2],this}toArray(e=[],t=0){return e[t]=this.r,e[t+1]=this.g,e[t+2]=this.b,e}fromBufferAttribute(e,t){return this.r=e.getX(t),this.g=e.getY(t),this.b=e.getZ(t),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const mt=new He;He.NAMES=Hc;let nf=0;class Bn extends Fn{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:nf++}),this.uuid=Pi(),this.name="",this.type="Material",this.blending=si,this.side=pn,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=sa,this.blendDst=aa,this.blendEquation=Cn,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new He(0,0,0),this.blendAlpha=0,this.depthFunc=dr,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=nl,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=Vn,this.stencilZFail=Vn,this.stencilZPass=Vn,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(e){this._alphaTest>0!=e>0&&this.version++,this._alphaTest=e}onBuild(){}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(e){if(e!==void 0)for(const t in e){const n=e[t];if(n===void 0){console.warn(`THREE.Material: parameter '${t}' has value of undefined.`);continue}const r=this[t];if(r===void 0){console.warn(`THREE.Material: '${t}' is not a property of THREE.${this.type}.`);continue}r&&r.isColor?r.set(n):r&&r.isVector3&&n&&n.isVector3?r.copy(n):this[t]=n}}toJSON(e){const t=e===void 0||typeof e=="string";t&&(e={textures:{},images:{}});const n={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};n.uuid=this.uuid,n.type=this.type,this.name!==""&&(n.name=this.name),this.color&&this.color.isColor&&(n.color=this.color.getHex()),this.roughness!==void 0&&(n.roughness=this.roughness),this.metalness!==void 0&&(n.metalness=this.metalness),this.sheen!==void 0&&(n.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(n.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(n.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(n.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(n.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(n.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(n.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(n.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(n.shininess=this.shininess),this.clearcoat!==void 0&&(n.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(n.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(n.clearcoatMap=this.clearcoatMap.toJSON(e).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(n.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(e).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(n.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(e).uuid,n.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.dispersion!==void 0&&(n.dispersion=this.dispersion),this.iridescence!==void 0&&(n.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(n.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(n.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(n.iridescenceMap=this.iridescenceMap.toJSON(e).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(n.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(e).uuid),this.anisotropy!==void 0&&(n.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(n.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(n.anisotropyMap=this.anisotropyMap.toJSON(e).uuid),this.map&&this.map.isTexture&&(n.map=this.map.toJSON(e).uuid),this.matcap&&this.matcap.isTexture&&(n.matcap=this.matcap.toJSON(e).uuid),this.alphaMap&&this.alphaMap.isTexture&&(n.alphaMap=this.alphaMap.toJSON(e).uuid),this.lightMap&&this.lightMap.isTexture&&(n.lightMap=this.lightMap.toJSON(e).uuid,n.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(n.aoMap=this.aoMap.toJSON(e).uuid,n.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(n.bumpMap=this.bumpMap.toJSON(e).uuid,n.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(n.normalMap=this.normalMap.toJSON(e).uuid,n.normalMapType=this.normalMapType,n.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(n.displacementMap=this.displacementMap.toJSON(e).uuid,n.displacementScale=this.displacementScale,n.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(n.roughnessMap=this.roughnessMap.toJSON(e).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(n.metalnessMap=this.metalnessMap.toJSON(e).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(n.emissiveMap=this.emissiveMap.toJSON(e).uuid),this.specularMap&&this.specularMap.isTexture&&(n.specularMap=this.specularMap.toJSON(e).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(n.specularIntensityMap=this.specularIntensityMap.toJSON(e).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(n.specularColorMap=this.specularColorMap.toJSON(e).uuid),this.envMap&&this.envMap.isTexture&&(n.envMap=this.envMap.toJSON(e).uuid,this.combine!==void 0&&(n.combine=this.combine)),this.envMapRotation!==void 0&&(n.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(n.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(n.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(n.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(n.gradientMap=this.gradientMap.toJSON(e).uuid),this.transmission!==void 0&&(n.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(n.transmissionMap=this.transmissionMap.toJSON(e).uuid),this.thickness!==void 0&&(n.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(n.thicknessMap=this.thicknessMap.toJSON(e).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(n.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(n.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(n.size=this.size),this.shadowSide!==null&&(n.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(n.sizeAttenuation=this.sizeAttenuation),this.blending!==si&&(n.blending=this.blending),this.side!==pn&&(n.side=this.side),this.vertexColors===!0&&(n.vertexColors=!0),this.opacity<1&&(n.opacity=this.opacity),this.transparent===!0&&(n.transparent=!0),this.blendSrc!==sa&&(n.blendSrc=this.blendSrc),this.blendDst!==aa&&(n.blendDst=this.blendDst),this.blendEquation!==Cn&&(n.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(n.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(n.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(n.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(n.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(n.blendAlpha=this.blendAlpha),this.depthFunc!==dr&&(n.depthFunc=this.depthFunc),this.depthTest===!1&&(n.depthTest=this.depthTest),this.depthWrite===!1&&(n.depthWrite=this.depthWrite),this.colorWrite===!1&&(n.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(n.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==nl&&(n.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(n.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(n.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==Vn&&(n.stencilFail=this.stencilFail),this.stencilZFail!==Vn&&(n.stencilZFail=this.stencilZFail),this.stencilZPass!==Vn&&(n.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(n.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(n.rotation=this.rotation),this.polygonOffset===!0&&(n.polygonOffset=!0),this.polygonOffsetFactor!==0&&(n.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(n.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(n.linewidth=this.linewidth),this.dashSize!==void 0&&(n.dashSize=this.dashSize),this.gapSize!==void 0&&(n.gapSize=this.gapSize),this.scale!==void 0&&(n.scale=this.scale),this.dithering===!0&&(n.dithering=!0),this.alphaTest>0&&(n.alphaTest=this.alphaTest),this.alphaHash===!0&&(n.alphaHash=!0),this.alphaToCoverage===!0&&(n.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(n.premultipliedAlpha=!0),this.forceSinglePass===!0&&(n.forceSinglePass=!0),this.wireframe===!0&&(n.wireframe=!0),this.wireframeLinewidth>1&&(n.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(n.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(n.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(n.flatShading=!0),this.visible===!1&&(n.visible=!1),this.toneMapped===!1&&(n.toneMapped=!1),this.fog===!1&&(n.fog=!1),Object.keys(this.userData).length>0&&(n.userData=this.userData);function r(s){const o=[];for(const a in s){const l=s[a];delete l.metadata,o.push(l)}return o}if(t){const s=r(e.textures),o=r(e.images);s.length>0&&(n.textures=s),o.length>0&&(n.images=o)}return n}clone(){return new this.constructor().copy(this)}copy(e){this.name=e.name,this.blending=e.blending,this.side=e.side,this.vertexColors=e.vertexColors,this.opacity=e.opacity,this.transparent=e.transparent,this.blendSrc=e.blendSrc,this.blendDst=e.blendDst,this.blendEquation=e.blendEquation,this.blendSrcAlpha=e.blendSrcAlpha,this.blendDstAlpha=e.blendDstAlpha,this.blendEquationAlpha=e.blendEquationAlpha,this.blendColor.copy(e.blendColor),this.blendAlpha=e.blendAlpha,this.depthFunc=e.depthFunc,this.depthTest=e.depthTest,this.depthWrite=e.depthWrite,this.stencilWriteMask=e.stencilWriteMask,this.stencilFunc=e.stencilFunc,this.stencilRef=e.stencilRef,this.stencilFuncMask=e.stencilFuncMask,this.stencilFail=e.stencilFail,this.stencilZFail=e.stencilZFail,this.stencilZPass=e.stencilZPass,this.stencilWrite=e.stencilWrite;const t=e.clippingPlanes;let n=null;if(t!==null){const r=t.length;n=new Array(r);for(let s=0;s!==r;++s)n[s]=t[s].clone()}return this.clippingPlanes=n,this.clipIntersection=e.clipIntersection,this.clipShadows=e.clipShadows,this.shadowSide=e.shadowSide,this.colorWrite=e.colorWrite,this.precision=e.precision,this.polygonOffset=e.polygonOffset,this.polygonOffsetFactor=e.polygonOffsetFactor,this.polygonOffsetUnits=e.polygonOffsetUnits,this.dithering=e.dithering,this.alphaTest=e.alphaTest,this.alphaHash=e.alphaHash,this.alphaToCoverage=e.alphaToCoverage,this.premultipliedAlpha=e.premultipliedAlpha,this.forceSinglePass=e.forceSinglePass,this.visible=e.visible,this.toneMapped=e.toneMapped,this.userData=JSON.parse(JSON.stringify(e.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(e){e===!0&&this.version++}}class Wc extends Bn{constructor(e){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new He(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Ht,this.combine=Cc,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.specularMap=e.specularMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.combine=e.combine,this.reflectivity=e.reflectivity,this.refractionRatio=e.refractionRatio,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.fog=e.fog,this}}const at=new G,Xi=new De;class Vt{constructor(e,t,n=!1){if(Array.isArray(e))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,this.name="",this.array=e,this.itemSize=t,this.count=e!==void 0?e.length/t:0,this.normalized=n,this.usage=il,this._updateRange={offset:0,count:-1},this.updateRanges=[],this.gpuType=un,this.version=0}onUploadCallback(){}set needsUpdate(e){e===!0&&this.version++}get updateRange(){return kc("THREE.BufferAttribute: updateRange() is deprecated and will be removed in r169. Use addUpdateRange() instead."),this._updateRange}setUsage(e){return this.usage=e,this}addUpdateRange(e,t){this.updateRanges.push({start:e,count:t})}clearUpdateRanges(){this.updateRanges.length=0}copy(e){return this.name=e.name,this.array=new e.array.constructor(e.array),this.itemSize=e.itemSize,this.count=e.count,this.normalized=e.normalized,this.usage=e.usage,this.gpuType=e.gpuType,this}copyAt(e,t,n){e*=this.itemSize,n*=t.itemSize;for(let r=0,s=this.itemSize;r<s;r++)this.array[e+r]=t.array[n+r];return this}copyArray(e){return this.array.set(e),this}applyMatrix3(e){if(this.itemSize===2)for(let t=0,n=this.count;t<n;t++)Xi.fromBufferAttribute(this,t),Xi.applyMatrix3(e),this.setXY(t,Xi.x,Xi.y);else if(this.itemSize===3)for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.applyMatrix3(e),this.setXYZ(t,at.x,at.y,at.z);return this}applyMatrix4(e){for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.applyMatrix4(e),this.setXYZ(t,at.x,at.y,at.z);return this}applyNormalMatrix(e){for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.applyNormalMatrix(e),this.setXYZ(t,at.x,at.y,at.z);return this}transformDirection(e){for(let t=0,n=this.count;t<n;t++)at.fromBufferAttribute(this,t),at.transformDirection(e),this.setXYZ(t,at.x,at.y,at.z);return this}set(e,t=0){return this.array.set(e,t),this}getComponent(e,t){let n=this.array[e*this.itemSize+t];return this.normalized&&(n=vi(n,this.array)),n}setComponent(e,t,n){return this.normalized&&(n=xt(n,this.array)),this.array[e*this.itemSize+t]=n,this}getX(e){let t=this.array[e*this.itemSize];return this.normalized&&(t=vi(t,this.array)),t}setX(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize]=t,this}getY(e){let t=this.array[e*this.itemSize+1];return this.normalized&&(t=vi(t,this.array)),t}setY(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize+1]=t,this}getZ(e){let t=this.array[e*this.itemSize+2];return this.normalized&&(t=vi(t,this.array)),t}setZ(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize+2]=t,this}getW(e){let t=this.array[e*this.itemSize+3];return this.normalized&&(t=vi(t,this.array)),t}setW(e,t){return this.normalized&&(t=xt(t,this.array)),this.array[e*this.itemSize+3]=t,this}setXY(e,t,n){return e*=this.itemSize,this.normalized&&(t=xt(t,this.array),n=xt(n,this.array)),this.array[e+0]=t,this.array[e+1]=n,this}setXYZ(e,t,n,r){return e*=this.itemSize,this.normalized&&(t=xt(t,this.array),n=xt(n,this.array),r=xt(r,this.array)),this.array[e+0]=t,this.array[e+1]=n,this.array[e+2]=r,this}setXYZW(e,t,n,r,s){return e*=this.itemSize,this.normalized&&(t=xt(t,this.array),n=xt(n,this.array),r=xt(r,this.array),s=xt(s,this.array)),this.array[e+0]=t,this.array[e+1]=n,this.array[e+2]=r,this.array[e+3]=s,this}onUpload(e){return this.onUploadCallback=e,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const e={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(e.name=this.name),this.usage!==il&&(e.usage=this.usage),e}}class Xc extends Vt{constructor(e,t,n){super(new Uint16Array(e),t,n)}}class qc extends Vt{constructor(e,t,n){super(new Uint32Array(e),t,n)}}class ft extends Vt{constructor(e,t,n){super(new Float32Array(e),t,n)}}let rf=0;const Ct=new Je,Xs=new dt,Zn=new G,Tt=new Li,Mi=new Li,ut=new G;class Lt extends Fn{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:rf++}),this.uuid=Pi(),this.name="",this.type="BufferGeometry",this.index=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(e){return Array.isArray(e)?this.index=new(Bc(e)?qc:Xc)(e,1):this.index=e,this}getAttribute(e){return this.attributes[e]}setAttribute(e,t){return this.attributes[e]=t,this}deleteAttribute(e){return delete this.attributes[e],this}hasAttribute(e){return this.attributes[e]!==void 0}addGroup(e,t,n=0){this.groups.push({start:e,count:t,materialIndex:n})}clearGroups(){this.groups=[]}setDrawRange(e,t){this.drawRange.start=e,this.drawRange.count=t}applyMatrix4(e){const t=this.attributes.position;t!==void 0&&(t.applyMatrix4(e),t.needsUpdate=!0);const n=this.attributes.normal;if(n!==void 0){const s=new ze().getNormalMatrix(e);n.applyNormalMatrix(s),n.needsUpdate=!0}const r=this.attributes.tangent;return r!==void 0&&(r.transformDirection(e),r.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(e){return Ct.makeRotationFromQuaternion(e),this.applyMatrix4(Ct),this}rotateX(e){return Ct.makeRotationX(e),this.applyMatrix4(Ct),this}rotateY(e){return Ct.makeRotationY(e),this.applyMatrix4(Ct),this}rotateZ(e){return Ct.makeRotationZ(e),this.applyMatrix4(Ct),this}translate(e,t,n){return Ct.makeTranslation(e,t,n),this.applyMatrix4(Ct),this}scale(e,t,n){return Ct.makeScale(e,t,n),this.applyMatrix4(Ct),this}lookAt(e){return Xs.lookAt(e),Xs.updateMatrix(),this.applyMatrix4(Xs.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(Zn).negate(),this.translate(Zn.x,Zn.y,Zn.z),this}setFromPoints(e){const t=[];for(let n=0,r=e.length;n<r;n++){const s=e[n];t.push(s.x,s.y,s.z||0)}return this.setAttribute("position",new ft(t,3)),this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new Li);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new G(-1/0,-1/0,-1/0),new G(1/0,1/0,1/0));return}if(e!==void 0){if(this.boundingBox.setFromBufferAttribute(e),t)for(let n=0,r=t.length;n<r;n++){const s=t[n];Tt.setFromBufferAttribute(s),this.morphTargetsRelative?(ut.addVectors(this.boundingBox.min,Tt.min),this.boundingBox.expandByPoint(ut),ut.addVectors(this.boundingBox.max,Tt.max),this.boundingBox.expandByPoint(ut)):(this.boundingBox.expandByPoint(Tt.min),this.boundingBox.expandByPoint(Tt.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new Ui);const e=this.attributes.position,t=this.morphAttributes.position;if(e&&e.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new G,1/0);return}if(e){const n=this.boundingSphere.center;if(Tt.setFromBufferAttribute(e),t)for(let s=0,o=t.length;s<o;s++){const a=t[s];Mi.setFromBufferAttribute(a),this.morphTargetsRelative?(ut.addVectors(Tt.min,Mi.min),Tt.expandByPoint(ut),ut.addVectors(Tt.max,Mi.max),Tt.expandByPoint(ut)):(Tt.expandByPoint(Mi.min),Tt.expandByPoint(Mi.max))}Tt.getCenter(n);let r=0;for(let s=0,o=e.count;s<o;s++)ut.fromBufferAttribute(e,s),r=Math.max(r,n.distanceToSquared(ut));if(t)for(let s=0,o=t.length;s<o;s++){const a=t[s],l=this.morphTargetsRelative;for(let c=0,u=a.count;c<u;c++)ut.fromBufferAttribute(a,c),l&&(Zn.fromBufferAttribute(e,c),ut.add(Zn)),r=Math.max(r,n.distanceToSquared(ut))}this.boundingSphere.radius=Math.sqrt(r),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const e=this.index,t=this.attributes;if(e===null||t.position===void 0||t.normal===void 0||t.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const n=t.position,r=t.normal,s=t.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new Vt(new Float32Array(4*n.count),4));const o=this.getAttribute("tangent"),a=[],l=[];for(let Y=0;Y<n.count;Y++)a[Y]=new G,l[Y]=new G;const c=new G,u=new G,h=new G,d=new De,f=new De,g=new De,x=new G,m=new G;function p(Y,E,b){c.fromBufferAttribute(n,Y),u.fromBufferAttribute(n,E),h.fromBufferAttribute(n,b),d.fromBufferAttribute(s,Y),f.fromBufferAttribute(s,E),g.fromBufferAttribute(s,b),u.sub(c),h.sub(c),f.sub(d),g.sub(d);const C=1/(f.x*g.y-g.x*f.y);isFinite(C)&&(x.copy(u).multiplyScalar(g.y).addScaledVector(h,-f.y).multiplyScalar(C),m.copy(h).multiplyScalar(f.x).addScaledVector(u,-g.x).multiplyScalar(C),a[Y].add(x),a[E].add(x),a[b].add(x),l[Y].add(m),l[E].add(m),l[b].add(m))}let w=this.groups;w.length===0&&(w=[{start:0,count:e.count}]);for(let Y=0,E=w.length;Y<E;++Y){const b=w[Y],C=b.start,H=b.count;for(let O=C,$=C+H;O<$;O+=3)p(e.getX(O+0),e.getX(O+1),e.getX(O+2))}const S=new G,A=new G,k=new G,I=new G;function P(Y){k.fromBufferAttribute(r,Y),I.copy(k);const E=a[Y];S.copy(E),S.sub(k.multiplyScalar(k.dot(E))).normalize(),A.crossVectors(I,E);const C=A.dot(l[Y])<0?-1:1;o.setXYZW(Y,S.x,S.y,S.z,C)}for(let Y=0,E=w.length;Y<E;++Y){const b=w[Y],C=b.start,H=b.count;for(let O=C,$=C+H;O<$;O+=3)P(e.getX(O+0)),P(e.getX(O+1)),P(e.getX(O+2))}}computeVertexNormals(){const e=this.index,t=this.getAttribute("position");if(t!==void 0){let n=this.getAttribute("normal");if(n===void 0)n=new Vt(new Float32Array(t.count*3),3),this.setAttribute("normal",n);else for(let d=0,f=n.count;d<f;d++)n.setXYZ(d,0,0,0);const r=new G,s=new G,o=new G,a=new G,l=new G,c=new G,u=new G,h=new G;if(e)for(let d=0,f=e.count;d<f;d+=3){const g=e.getX(d+0),x=e.getX(d+1),m=e.getX(d+2);r.fromBufferAttribute(t,g),s.fromBufferAttribute(t,x),o.fromBufferAttribute(t,m),u.subVectors(o,s),h.subVectors(r,s),u.cross(h),a.fromBufferAttribute(n,g),l.fromBufferAttribute(n,x),c.fromBufferAttribute(n,m),a.add(u),l.add(u),c.add(u),n.setXYZ(g,a.x,a.y,a.z),n.setXYZ(x,l.x,l.y,l.z),n.setXYZ(m,c.x,c.y,c.z)}else for(let d=0,f=t.count;d<f;d+=3)r.fromBufferAttribute(t,d+0),s.fromBufferAttribute(t,d+1),o.fromBufferAttribute(t,d+2),u.subVectors(o,s),h.subVectors(r,s),u.cross(h),n.setXYZ(d+0,u.x,u.y,u.z),n.setXYZ(d+1,u.x,u.y,u.z),n.setXYZ(d+2,u.x,u.y,u.z);this.normalizeNormals(),n.needsUpdate=!0}}normalizeNormals(){const e=this.attributes.normal;for(let t=0,n=e.count;t<n;t++)ut.fromBufferAttribute(e,t),ut.normalize(),e.setXYZ(t,ut.x,ut.y,ut.z)}toNonIndexed(){function e(a,l){const c=a.array,u=a.itemSize,h=a.normalized,d=new c.constructor(l.length*u);let f=0,g=0;for(let x=0,m=l.length;x<m;x++){a.isInterleavedBufferAttribute?f=l[x]*a.data.stride+a.offset:f=l[x]*u;for(let p=0;p<u;p++)d[g++]=c[f++]}return new Vt(d,u,h)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const t=new Lt,n=this.index.array,r=this.attributes;for(const a in r){const l=r[a],c=e(l,n);t.setAttribute(a,c)}const s=this.morphAttributes;for(const a in s){const l=[],c=s[a];for(let u=0,h=c.length;u<h;u++){const d=c[u],f=e(d,n);l.push(f)}t.morphAttributes[a]=l}t.morphTargetsRelative=this.morphTargetsRelative;const o=this.groups;for(let a=0,l=o.length;a<l;a++){const c=o[a];t.addGroup(c.start,c.count,c.materialIndex)}return t}toJSON(){const e={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(e.uuid=this.uuid,e.type=this.type,this.name!==""&&(e.name=this.name),Object.keys(this.userData).length>0&&(e.userData=this.userData),this.parameters!==void 0){const l=this.parameters;for(const c in l)l[c]!==void 0&&(e[c]=l[c]);return e}e.data={attributes:{}};const t=this.index;t!==null&&(e.data.index={type:t.array.constructor.name,array:Array.prototype.slice.call(t.array)});const n=this.attributes;for(const l in n){const c=n[l];e.data.attributes[l]=c.toJSON(e.data)}const r={};let s=!1;for(const l in this.morphAttributes){const c=this.morphAttributes[l],u=[];for(let h=0,d=c.length;h<d;h++){const f=c[h];u.push(f.toJSON(e.data))}u.length>0&&(r[l]=u,s=!0)}s&&(e.data.morphAttributes=r,e.data.morphTargetsRelative=this.morphTargetsRelative);const o=this.groups;o.length>0&&(e.data.groups=JSON.parse(JSON.stringify(o)));const a=this.boundingSphere;return a!==null&&(e.data.boundingSphere={center:a.center.toArray(),radius:a.radius}),e}clone(){return new this.constructor().copy(this)}copy(e){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const t={};this.name=e.name;const n=e.index;n!==null&&this.setIndex(n.clone(t));const r=e.attributes;for(const c in r){const u=r[c];this.setAttribute(c,u.clone(t))}const s=e.morphAttributes;for(const c in s){const u=[],h=s[c];for(let d=0,f=h.length;d<f;d++)u.push(h[d].clone(t));this.morphAttributes[c]=u}this.morphTargetsRelative=e.morphTargetsRelative;const o=e.groups;for(let c=0,u=o.length;c<u;c++){const h=o[c];this.addGroup(h.start,h.count,h.materialIndex)}const a=e.boundingBox;a!==null&&(this.boundingBox=a.clone());const l=e.boundingSphere;return l!==null&&(this.boundingSphere=l.clone()),this.drawRange.start=e.drawRange.start,this.drawRange.count=e.drawRange.count,this.userData=e.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const _l=new Je,bn=new wr,qi=new Ui,vl=new G,Jn=new G,Qn=new G,ei=new G,qs=new G,Yi=new G,ji=new De,$i=new De,Ki=new De,xl=new G,yl=new G,Sl=new G,Zi=new G,Ji=new G;class Ot extends dt{constructor(e=new Lt,t=new Wc){super(),this.isMesh=!0,this.type="Mesh",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),e.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=e.morphTargetInfluences.slice()),e.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},e.morphTargetDictionary)),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}updateMorphTargets(){const t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){const r=t[n[0]];if(r!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,o=r.length;s<o;s++){const a=r[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=s}}}}getVertexPosition(e,t){const n=this.geometry,r=n.attributes.position,s=n.morphAttributes.position,o=n.morphTargetsRelative;t.fromBufferAttribute(r,e);const a=this.morphTargetInfluences;if(s&&a){Yi.set(0,0,0);for(let l=0,c=s.length;l<c;l++){const u=a[l],h=s[l];u!==0&&(qs.fromBufferAttribute(h,e),o?Yi.addScaledVector(qs,u):Yi.addScaledVector(qs.sub(t),u))}t.add(Yi)}return t}raycast(e,t){const n=this.geometry,r=this.material,s=this.matrixWorld;r!==void 0&&(n.boundingSphere===null&&n.computeBoundingSphere(),qi.copy(n.boundingSphere),qi.applyMatrix4(s),bn.copy(e.ray).recast(e.near),!(qi.containsPoint(bn.origin)===!1&&(bn.intersectSphere(qi,vl)===null||bn.origin.distanceToSquared(vl)>(e.far-e.near)**2))&&(_l.copy(s).invert(),bn.copy(e.ray).applyMatrix4(_l),!(n.boundingBox!==null&&bn.intersectsBox(n.boundingBox)===!1)&&this._computeIntersections(e,t,bn)))}_computeIntersections(e,t,n){let r;const s=this.geometry,o=this.material,a=s.index,l=s.attributes.position,c=s.attributes.uv,u=s.attributes.uv1,h=s.attributes.normal,d=s.groups,f=s.drawRange;if(a!==null)if(Array.isArray(o))for(let g=0,x=d.length;g<x;g++){const m=d[g],p=o[m.materialIndex],w=Math.max(m.start,f.start),S=Math.min(a.count,Math.min(m.start+m.count,f.start+f.count));for(let A=w,k=S;A<k;A+=3){const I=a.getX(A),P=a.getX(A+1),Y=a.getX(A+2);r=Qi(this,p,e,n,c,u,h,I,P,Y),r&&(r.faceIndex=Math.floor(A/3),r.face.materialIndex=m.materialIndex,t.push(r))}}else{const g=Math.max(0,f.start),x=Math.min(a.count,f.start+f.count);for(let m=g,p=x;m<p;m+=3){const w=a.getX(m),S=a.getX(m+1),A=a.getX(m+2);r=Qi(this,o,e,n,c,u,h,w,S,A),r&&(r.faceIndex=Math.floor(m/3),t.push(r))}}else if(l!==void 0)if(Array.isArray(o))for(let g=0,x=d.length;g<x;g++){const m=d[g],p=o[m.materialIndex],w=Math.max(m.start,f.start),S=Math.min(l.count,Math.min(m.start+m.count,f.start+f.count));for(let A=w,k=S;A<k;A+=3){const I=A,P=A+1,Y=A+2;r=Qi(this,p,e,n,c,u,h,I,P,Y),r&&(r.faceIndex=Math.floor(A/3),r.face.materialIndex=m.materialIndex,t.push(r))}}else{const g=Math.max(0,f.start),x=Math.min(l.count,f.start+f.count);for(let m=g,p=x;m<p;m+=3){const w=m,S=m+1,A=m+2;r=Qi(this,o,e,n,c,u,h,w,S,A),r&&(r.faceIndex=Math.floor(m/3),t.push(r))}}}}function sf(i,e,t,n,r,s,o,a){let l;if(e.side===yt?l=n.intersectTriangle(o,s,r,!0,a):l=n.intersectTriangle(r,s,o,e.side===pn,a),l===null)return null;Ji.copy(a),Ji.applyMatrix4(i.matrixWorld);const c=t.ray.origin.distanceTo(Ji);return c<t.near||c>t.far?null:{distance:c,point:Ji.clone(),object:i}}function Qi(i,e,t,n,r,s,o,a,l,c){i.getVertexPosition(a,Jn),i.getVertexPosition(l,Qn),i.getVertexPosition(c,ei);const u=sf(i,e,t,n,Jn,Qn,ei,Zi);if(u){r&&(ji.fromBufferAttribute(r,a),$i.fromBufferAttribute(r,l),Ki.fromBufferAttribute(r,c),u.uv=kt.getInterpolation(Zi,Jn,Qn,ei,ji,$i,Ki,new De)),s&&(ji.fromBufferAttribute(s,a),$i.fromBufferAttribute(s,l),Ki.fromBufferAttribute(s,c),u.uv1=kt.getInterpolation(Zi,Jn,Qn,ei,ji,$i,Ki,new De)),o&&(xl.fromBufferAttribute(o,a),yl.fromBufferAttribute(o,l),Sl.fromBufferAttribute(o,c),u.normal=kt.getInterpolation(Zi,Jn,Qn,ei,xl,yl,Sl,new G),u.normal.dot(n.direction)>0&&u.normal.multiplyScalar(-1));const h={a,b:l,c,normal:new G,materialIndex:0};kt.getNormal(Jn,Qn,ei,h.normal),u.face=h}return u}class Dn extends Lt{constructor(e=1,t=1,n=1,r=1,s=1,o=1){super(),this.type="BoxGeometry",this.parameters={width:e,height:t,depth:n,widthSegments:r,heightSegments:s,depthSegments:o};const a=this;r=Math.floor(r),s=Math.floor(s),o=Math.floor(o);const l=[],c=[],u=[],h=[];let d=0,f=0;g("z","y","x",-1,-1,n,t,e,o,s,0),g("z","y","x",1,-1,n,t,-e,o,s,1),g("x","z","y",1,1,e,n,t,r,o,2),g("x","z","y",1,-1,e,n,-t,r,o,3),g("x","y","z",1,-1,e,t,n,r,s,4),g("x","y","z",-1,-1,e,t,-n,r,s,5),this.setIndex(l),this.setAttribute("position",new ft(c,3)),this.setAttribute("normal",new ft(u,3)),this.setAttribute("uv",new ft(h,2));function g(x,m,p,w,S,A,k,I,P,Y,E){const b=A/P,C=k/Y,H=A/2,O=k/2,$=I/2,z=P+1,q=Y+1;let te=0,v=0;const T=new G;for(let U=0;U<q;U++){const N=U*C-O;for(let V=0;V<z;V++){const K=V*b-H;T[x]=K*w,T[m]=N*S,T[p]=$,c.push(T.x,T.y,T.z),T[x]=0,T[m]=0,T[p]=I>0?1:-1,u.push(T.x,T.y,T.z),h.push(V/P),h.push(1-U/Y),te+=1}}for(let U=0;U<Y;U++)for(let N=0;N<P;N++){const V=d+N+z*U,K=d+N+z*(U+1),D=d+(N+1)+z*(U+1),B=d+(N+1)+z*U;l.push(V,K,B),l.push(K,D,B),v+=6}a.addGroup(f,v,E),f+=v,d+=te}}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new Dn(e.width,e.height,e.depth,e.widthSegments,e.heightSegments,e.depthSegments)}}function fi(i){const e={};for(const t in i){e[t]={};for(const n in i[t]){const r=i[t][n];r&&(r.isColor||r.isMatrix3||r.isMatrix4||r.isVector2||r.isVector3||r.isVector4||r.isTexture||r.isQuaternion)?r.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),e[t][n]=null):e[t][n]=r.clone():Array.isArray(r)?e[t][n]=r.slice():e[t][n]=r}}return e}function _t(i){const e={};for(let t=0;t<i.length;t++){const n=fi(i[t]);for(const r in n)e[r]=n[r]}return e}function af(i){const e=[];for(let t=0;t<i.length;t++)e.push(i[t].clone());return e}function Yc(i){const e=i.getRenderTarget();return e===null?i.outputColorSpace:e.isXRRenderTarget===!0?e.texture.colorSpace:je.workingColorSpace}const of={clone:fi,merge:_t};var lf=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,cf=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class gn extends Bn{constructor(e){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=lf,this.fragmentShader=cf,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,e!==void 0&&this.setValues(e)}copy(e){return super.copy(e),this.fragmentShader=e.fragmentShader,this.vertexShader=e.vertexShader,this.uniforms=fi(e.uniforms),this.uniformsGroups=af(e.uniformsGroups),this.defines=Object.assign({},e.defines),this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.fog=e.fog,this.lights=e.lights,this.clipping=e.clipping,this.extensions=Object.assign({},e.extensions),this.glslVersion=e.glslVersion,this}toJSON(e){const t=super.toJSON(e);t.glslVersion=this.glslVersion,t.uniforms={};for(const r in this.uniforms){const o=this.uniforms[r].value;o&&o.isTexture?t.uniforms[r]={type:"t",value:o.toJSON(e).uuid}:o&&o.isColor?t.uniforms[r]={type:"c",value:o.getHex()}:o&&o.isVector2?t.uniforms[r]={type:"v2",value:o.toArray()}:o&&o.isVector3?t.uniforms[r]={type:"v3",value:o.toArray()}:o&&o.isVector4?t.uniforms[r]={type:"v4",value:o.toArray()}:o&&o.isMatrix3?t.uniforms[r]={type:"m3",value:o.toArray()}:o&&o.isMatrix4?t.uniforms[r]={type:"m4",value:o.toArray()}:t.uniforms[r]={value:o}}Object.keys(this.defines).length>0&&(t.defines=this.defines),t.vertexShader=this.vertexShader,t.fragmentShader=this.fragmentShader,t.lights=this.lights,t.clipping=this.clipping;const n={};for(const r in this.extensions)this.extensions[r]===!0&&(n[r]=!0);return Object.keys(n).length>0&&(t.extensions=n),t}}class jc extends dt{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new Je,this.projectionMatrix=new Je,this.projectionMatrixInverse=new Je,this.coordinateSystem=Jt}copy(e,t){return super.copy(e,t),this.matrixWorldInverse.copy(e.matrixWorldInverse),this.projectionMatrix.copy(e.projectionMatrix),this.projectionMatrixInverse.copy(e.projectionMatrixInverse),this.coordinateSystem=e.coordinateSystem,this}getWorldDirection(e){return super.getWorldDirection(e).negate()}updateMatrixWorld(e){super.updateMatrixWorld(e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(e,t){super.updateWorldMatrix(e,t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}}const on=new G,Ml=new De,bl=new De;class Rt extends jc{constructor(e=50,t=1,n=.1,r=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=e,this.zoom=1,this.near=n,this.far=r,this.focus=10,this.aspect=t,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.fov=e.fov,this.zoom=e.zoom,this.near=e.near,this.far=e.far,this.focus=e.focus,this.aspect=e.aspect,this.view=e.view===null?null:Object.assign({},e.view),this.filmGauge=e.filmGauge,this.filmOffset=e.filmOffset,this}setFocalLength(e){const t=.5*this.getFilmHeight()/e;this.fov=ha*2*Math.atan(t),this.updateProjectionMatrix()}getFocalLength(){const e=Math.tan(ur*.5*this.fov);return .5*this.getFilmHeight()/e}getEffectiveFOV(){return ha*2*Math.atan(Math.tan(ur*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(e,t,n){on.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),t.set(on.x,on.y).multiplyScalar(-e/on.z),on.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),n.set(on.x,on.y).multiplyScalar(-e/on.z)}getViewSize(e,t){return this.getViewBounds(e,Ml,bl),t.subVectors(bl,Ml)}setViewOffset(e,t,n,r,s,o){this.aspect=e/t,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=n,this.view.offsetY=r,this.view.width=s,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=this.near;let t=e*Math.tan(ur*.5*this.fov)/this.zoom,n=2*t,r=this.aspect*n,s=-.5*r;const o=this.view;if(this.view!==null&&this.view.enabled){const l=o.fullWidth,c=o.fullHeight;s+=o.offsetX*r/l,t-=o.offsetY*n/c,r*=o.width/l,n*=o.height/c}const a=this.filmOffset;a!==0&&(s+=e*a/this.getFilmWidth()),this.projectionMatrix.makePerspective(s,s+r,t,t-n,e,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.fov=this.fov,t.object.zoom=this.zoom,t.object.near=this.near,t.object.far=this.far,t.object.focus=this.focus,t.object.aspect=this.aspect,this.view!==null&&(t.object.view=Object.assign({},this.view)),t.object.filmGauge=this.filmGauge,t.object.filmOffset=this.filmOffset,t}}const ti=-90,ni=1;class uf extends dt{constructor(e,t,n){super(),this.type="CubeCamera",this.renderTarget=n,this.coordinateSystem=null,this.activeMipmapLevel=0;const r=new Rt(ti,ni,e,t);r.layers=this.layers,this.add(r);const s=new Rt(ti,ni,e,t);s.layers=this.layers,this.add(s);const o=new Rt(ti,ni,e,t);o.layers=this.layers,this.add(o);const a=new Rt(ti,ni,e,t);a.layers=this.layers,this.add(a);const l=new Rt(ti,ni,e,t);l.layers=this.layers,this.add(l);const c=new Rt(ti,ni,e,t);c.layers=this.layers,this.add(c)}updateCoordinateSystem(){const e=this.coordinateSystem,t=this.children.concat(),[n,r,s,o,a,l]=t;for(const c of t)this.remove(c);if(e===Jt)n.up.set(0,1,0),n.lookAt(1,0,0),r.up.set(0,1,0),r.lookAt(-1,0,0),s.up.set(0,0,-1),s.lookAt(0,1,0),o.up.set(0,0,1),o.lookAt(0,-1,0),a.up.set(0,1,0),a.lookAt(0,0,1),l.up.set(0,1,0),l.lookAt(0,0,-1);else if(e===_r)n.up.set(0,-1,0),n.lookAt(-1,0,0),r.up.set(0,-1,0),r.lookAt(1,0,0),s.up.set(0,0,1),s.lookAt(0,1,0),o.up.set(0,0,-1),o.lookAt(0,-1,0),a.up.set(0,-1,0),a.lookAt(0,0,1),l.up.set(0,-1,0),l.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+e);for(const c of t)this.add(c),c.updateMatrixWorld()}update(e,t){this.parent===null&&this.updateMatrixWorld();const{renderTarget:n,activeMipmapLevel:r}=this;this.coordinateSystem!==e.coordinateSystem&&(this.coordinateSystem=e.coordinateSystem,this.updateCoordinateSystem());const[s,o,a,l,c,u]=this.children,h=e.getRenderTarget(),d=e.getActiveCubeFace(),f=e.getActiveMipmapLevel(),g=e.xr.enabled;e.xr.enabled=!1;const x=n.texture.generateMipmaps;n.texture.generateMipmaps=!1,e.setRenderTarget(n,0,r),e.render(t,s),e.setRenderTarget(n,1,r),e.render(t,o),e.setRenderTarget(n,2,r),e.render(t,a),e.setRenderTarget(n,3,r),e.render(t,l),e.setRenderTarget(n,4,r),e.render(t,c),n.texture.generateMipmaps=x,e.setRenderTarget(n,5,r),e.render(t,u),e.setRenderTarget(h,d,f),e.xr.enabled=g,n.texture.needsPMREMUpdate=!0}}class $c extends St{constructor(e,t,n,r,s,o,a,l,c,u){e=e!==void 0?e:[],t=t!==void 0?t:li,super(e,t,n,r,s,o,a,l,c,u),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(e){this.image=e}}class hf extends Un{constructor(e=1,t={}){super(e,e,t),this.isWebGLCubeRenderTarget=!0;const n={width:e,height:e,depth:1},r=[n,n,n,n,n,n];this.texture=new $c(r,t.mapping,t.wrapS,t.wrapT,t.magFilter,t.minFilter,t.format,t.type,t.anisotropy,t.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=t.generateMipmaps!==void 0?t.generateMipmaps:!1,this.texture.minFilter=t.minFilter!==void 0?t.minFilter:Nt}fromEquirectangularTexture(e,t){this.texture.type=t.type,this.texture.colorSpace=t.colorSpace,this.texture.generateMipmaps=t.generateMipmaps,this.texture.minFilter=t.minFilter,this.texture.magFilter=t.magFilter;const n={uniforms:{tEquirect:{value:null}},vertexShader:`

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
			`},r=new Dn(5,5,5),s=new gn({name:"CubemapFromEquirect",uniforms:fi(n.uniforms),vertexShader:n.vertexShader,fragmentShader:n.fragmentShader,side:yt,blending:dn});s.uniforms.tEquirect.value=t;const o=new Ot(r,s),a=t.minFilter;return t.minFilter===Ln&&(t.minFilter=Nt),new uf(1,10,this).update(e,o),t.minFilter=a,o.geometry.dispose(),o.material.dispose(),this}clear(e,t,n,r){const s=e.getRenderTarget();for(let o=0;o<6;o++)e.setRenderTarget(this,o),e.clear(t,n,r);e.setRenderTarget(s)}}const Ys=new G,df=new G,ff=new ze;class ln{constructor(e=new G(1,0,0),t=0){this.isPlane=!0,this.normal=e,this.constant=t}set(e,t){return this.normal.copy(e),this.constant=t,this}setComponents(e,t,n,r){return this.normal.set(e,t,n),this.constant=r,this}setFromNormalAndCoplanarPoint(e,t){return this.normal.copy(e),this.constant=-t.dot(this.normal),this}setFromCoplanarPoints(e,t,n){const r=Ys.subVectors(n,t).cross(df.subVectors(e,t)).normalize();return this.setFromNormalAndCoplanarPoint(r,e),this}copy(e){return this.normal.copy(e.normal),this.constant=e.constant,this}normalize(){const e=1/this.normal.length();return this.normal.multiplyScalar(e),this.constant*=e,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(e){return this.normal.dot(e)+this.constant}distanceToSphere(e){return this.distanceToPoint(e.center)-e.radius}projectPoint(e,t){return t.copy(e).addScaledVector(this.normal,-this.distanceToPoint(e))}intersectLine(e,t){const n=e.delta(Ys),r=this.normal.dot(n);if(r===0)return this.distanceToPoint(e.start)===0?t.copy(e.start):null;const s=-(e.start.dot(this.normal)+this.constant)/r;return s<0||s>1?null:t.copy(e.start).addScaledVector(n,s)}intersectsLine(e){const t=this.distanceToPoint(e.start),n=this.distanceToPoint(e.end);return t<0&&n>0||n<0&&t>0}intersectsBox(e){return e.intersectsPlane(this)}intersectsSphere(e){return e.intersectsPlane(this)}coplanarPoint(e){return e.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(e,t){const n=t||ff.getNormalMatrix(e),r=this.coplanarPoint(Ys).applyMatrix4(e),s=this.normal.applyMatrix3(n).normalize();return this.constant=-r.dot(s),this}translate(e){return this.constant-=e.dot(this.normal),this}equals(e){return e.normal.equals(this.normal)&&e.constant===this.constant}clone(){return new this.constructor().copy(this)}}const En=new Ui,er=new G;class Sa{constructor(e=new ln,t=new ln,n=new ln,r=new ln,s=new ln,o=new ln){this.planes=[e,t,n,r,s,o]}set(e,t,n,r,s,o){const a=this.planes;return a[0].copy(e),a[1].copy(t),a[2].copy(n),a[3].copy(r),a[4].copy(s),a[5].copy(o),this}copy(e){const t=this.planes;for(let n=0;n<6;n++)t[n].copy(e.planes[n]);return this}setFromProjectionMatrix(e,t=Jt){const n=this.planes,r=e.elements,s=r[0],o=r[1],a=r[2],l=r[3],c=r[4],u=r[5],h=r[6],d=r[7],f=r[8],g=r[9],x=r[10],m=r[11],p=r[12],w=r[13],S=r[14],A=r[15];if(n[0].setComponents(l-s,d-c,m-f,A-p).normalize(),n[1].setComponents(l+s,d+c,m+f,A+p).normalize(),n[2].setComponents(l+o,d+u,m+g,A+w).normalize(),n[3].setComponents(l-o,d-u,m-g,A-w).normalize(),n[4].setComponents(l-a,d-h,m-x,A-S).normalize(),t===Jt)n[5].setComponents(l+a,d+h,m+x,A+S).normalize();else if(t===_r)n[5].setComponents(a,h,x,S).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+t);return this}intersectsObject(e){if(e.boundingSphere!==void 0)e.boundingSphere===null&&e.computeBoundingSphere(),En.copy(e.boundingSphere).applyMatrix4(e.matrixWorld);else{const t=e.geometry;t.boundingSphere===null&&t.computeBoundingSphere(),En.copy(t.boundingSphere).applyMatrix4(e.matrixWorld)}return this.intersectsSphere(En)}intersectsSprite(e){return En.center.set(0,0,0),En.radius=.7071067811865476,En.applyMatrix4(e.matrixWorld),this.intersectsSphere(En)}intersectsSphere(e){const t=this.planes,n=e.center,r=-e.radius;for(let s=0;s<6;s++)if(t[s].distanceToPoint(n)<r)return!1;return!0}intersectsBox(e){const t=this.planes;for(let n=0;n<6;n++){const r=t[n];if(er.x=r.normal.x>0?e.max.x:e.min.x,er.y=r.normal.y>0?e.max.y:e.min.y,er.z=r.normal.z>0?e.max.z:e.min.z,r.distanceToPoint(er)<0)return!1}return!0}containsPoint(e){const t=this.planes;for(let n=0;n<6;n++)if(t[n].distanceToPoint(e)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}function Kc(){let i=null,e=!1,t=null,n=null;function r(s,o){t(s,o),n=i.requestAnimationFrame(r)}return{start:function(){e!==!0&&t!==null&&(n=i.requestAnimationFrame(r),e=!0)},stop:function(){i.cancelAnimationFrame(n),e=!1},setAnimationLoop:function(s){t=s},setContext:function(s){i=s}}}function pf(i){const e=new WeakMap;function t(a,l){const c=a.array,u=a.usage,h=c.byteLength,d=i.createBuffer();i.bindBuffer(l,d),i.bufferData(l,c,u),a.onUploadCallback();let f;if(c instanceof Float32Array)f=i.FLOAT;else if(c instanceof Uint16Array)a.isFloat16BufferAttribute?f=i.HALF_FLOAT:f=i.UNSIGNED_SHORT;else if(c instanceof Int16Array)f=i.SHORT;else if(c instanceof Uint32Array)f=i.UNSIGNED_INT;else if(c instanceof Int32Array)f=i.INT;else if(c instanceof Int8Array)f=i.BYTE;else if(c instanceof Uint8Array)f=i.UNSIGNED_BYTE;else if(c instanceof Uint8ClampedArray)f=i.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+c);return{buffer:d,type:f,bytesPerElement:c.BYTES_PER_ELEMENT,version:a.version,size:h}}function n(a,l,c){const u=l.array,h=l._updateRange,d=l.updateRanges;if(i.bindBuffer(c,a),h.count===-1&&d.length===0&&i.bufferSubData(c,0,u),d.length!==0){for(let f=0,g=d.length;f<g;f++){const x=d[f];i.bufferSubData(c,x.start*u.BYTES_PER_ELEMENT,u,x.start,x.count)}l.clearUpdateRanges()}h.count!==-1&&(i.bufferSubData(c,h.offset*u.BYTES_PER_ELEMENT,u,h.offset,h.count),h.count=-1),l.onUploadCallback()}function r(a){return a.isInterleavedBufferAttribute&&(a=a.data),e.get(a)}function s(a){a.isInterleavedBufferAttribute&&(a=a.data);const l=e.get(a);l&&(i.deleteBuffer(l.buffer),e.delete(a))}function o(a,l){if(a.isGLBufferAttribute){const u=e.get(a);(!u||u.version<a.version)&&e.set(a,{buffer:a.buffer,type:a.type,bytesPerElement:a.elementSize,version:a.version});return}a.isInterleavedBufferAttribute&&(a=a.data);const c=e.get(a);if(c===void 0)e.set(a,t(a,l));else if(c.version<a.version){if(c.size!==a.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");n(c.buffer,a,l),c.version=a.version}}return{get:r,remove:s,update:o}}class Tr extends Lt{constructor(e=1,t=1,n=1,r=1){super(),this.type="PlaneGeometry",this.parameters={width:e,height:t,widthSegments:n,heightSegments:r};const s=e/2,o=t/2,a=Math.floor(n),l=Math.floor(r),c=a+1,u=l+1,h=e/a,d=t/l,f=[],g=[],x=[],m=[];for(let p=0;p<u;p++){const w=p*d-o;for(let S=0;S<c;S++){const A=S*h-s;g.push(A,-w,0),x.push(0,0,1),m.push(S/a),m.push(1-p/l)}}for(let p=0;p<l;p++)for(let w=0;w<a;w++){const S=w+c*p,A=w+c*(p+1),k=w+1+c*(p+1),I=w+1+c*p;f.push(S,A,I),f.push(A,k,I)}this.setIndex(f),this.setAttribute("position",new ft(g,3)),this.setAttribute("normal",new ft(x,3)),this.setAttribute("uv",new ft(m,2))}copy(e){return super.copy(e),this.parameters=Object.assign({},e.parameters),this}static fromJSON(e){return new Tr(e.width,e.height,e.widthSegments,e.heightSegments)}}var mf=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,gf=`#ifdef USE_ALPHAHASH
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
#endif`,_f=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,vf=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,xf=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,yf=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,Sf=`#ifdef USE_AOMAP
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
#endif`,Mf=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,bf=`#ifdef USE_BATCHING
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
#endif`,Ef=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( batchId );
#endif`,wf=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,Tf=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,Af=`float G_BlinnPhong_Implicit( ) {
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
} // validated`,Cf=`#ifdef USE_IRIDESCENCE
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
#endif`,Rf=`#ifdef USE_BUMPMAP
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
#endif`,Pf=`#if NUM_CLIPPING_PLANES > 0
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
#endif`,Lf=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,Uf=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,Df=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,If=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,Nf=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,Of=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,Ff=`#if defined( USE_COLOR_ALPHA )
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
#endif`,Bf=`#define PI 3.141592653589793
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
} // validated`,kf=`#ifdef ENVMAP_TYPE_CUBE_UV
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
#endif`,zf=`vec3 transformedNormal = objectNormal;
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
#endif`,Vf=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,Gf=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,Hf=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,Wf=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,Xf="gl_FragColor = linearToOutputTexel( gl_FragColor );",qf=`
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
}`,Yf=`#ifdef USE_ENVMAP
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
#endif`,jf=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,$f=`#ifdef USE_ENVMAP
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
#endif`,Kf=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,Zf=`#ifdef USE_ENVMAP
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
#endif`,Jf=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,Qf=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,ep=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,tp=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,np=`#ifdef USE_GRADIENTMAP
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
}`,ip=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,rp=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,sp=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,ap=`uniform bool receiveShadow;
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
#endif`,op=`#ifdef USE_ENVMAP
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
#endif`,lp=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,cp=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,up=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,hp=`varying vec3 vViewPosition;
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
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,dp=`PhysicalMaterial material;
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
#endif`,fp=`struct PhysicalMaterial {
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
}`,pp=`
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
#endif`,mp=`#if defined( RE_IndirectDiffuse )
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
#endif`,gp=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,_p=`#if defined( USE_LOGDEPTHBUF )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,vp=`#if defined( USE_LOGDEPTHBUF )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,xp=`#ifdef USE_LOGDEPTHBUF
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,yp=`#ifdef USE_LOGDEPTHBUF
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,Sp=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = vec4( mix( pow( sampledDiffuseColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), sampledDiffuseColor.rgb * 0.0773993808, vec3( lessThanEqual( sampledDiffuseColor.rgb, vec3( 0.04045 ) ) ) ), sampledDiffuseColor.w );
	
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,Mp=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,bp=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
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
#endif`,Ep=`#if defined( USE_POINTS_UV )
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
#endif`,wp=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,Tp=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,Ap=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,Cp=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,Rp=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,Pp=`#ifdef USE_MORPHTARGETS
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
#endif`,Lp=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,Up=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
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
vec3 nonPerturbedNormal = normal;`,Dp=`#ifdef USE_NORMALMAP_OBJECTSPACE
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
#endif`,Ip=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Np=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,Op=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,Fp=`#ifdef USE_NORMALMAP
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
#endif`,Bp=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,kp=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,zp=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,Vp=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,Gp=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,Hp=`vec3 packNormalToRGB( const in vec3 normal ) {
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
}`,Wp=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,Xp=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,qp=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,Yp=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,jp=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,$p=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,Kp=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,Zp=`#if NUM_SPOT_LIGHT_COORDS > 0
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
#endif`,Jp=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
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
#endif`,Qp=`float getShadowMask() {
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
}`,em=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,tm=`#ifdef USE_SKINNING
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
#endif`,nm=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,im=`#ifdef USE_SKINNING
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
#endif`,rm=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,sm=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,am=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,om=`#ifndef saturate
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
vec3 CustomToneMapping( vec3 color ) { return color; }`,lm=`#ifdef USE_TRANSMISSION
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
#endif`,cm=`#ifdef USE_TRANSMISSION
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
#endif`,um=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,hm=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,dm=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
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
#endif`,fm=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const pm=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,mm=`uniform sampler2D t2D;
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
}`,gm=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,_m=`#ifdef ENVMAP_TYPE_CUBE
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
}`,vm=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,xm=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,ym=`#include <common>
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
}`,Sm=`#if DEPTH_PACKING == 3200
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
}`,Mm=`#define DISTANCE
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
}`,bm=`#define DISTANCE
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
}`,Em=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,wm=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,Tm=`uniform float scale;
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
}`,Am=`uniform vec3 diffuse;
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
}`,Cm=`#include <common>
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
}`,Rm=`uniform vec3 diffuse;
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
}`,Pm=`#define LAMBERT
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
}`,Lm=`#define LAMBERT
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
}`,Um=`#define MATCAP
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
}`,Dm=`#define MATCAP
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
}`,Im=`#define NORMAL
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
}`,Nm=`#define NORMAL
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
}`,Om=`#define PHONG
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
}`,Fm=`#define PHONG
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
}`,Bm=`#define STANDARD
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
}`,km=`#define STANDARD
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
}`,zm=`#define TOON
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
}`,Vm=`#define TOON
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
}`,Gm=`uniform float size;
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
}`,Hm=`uniform vec3 diffuse;
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
}`,Wm=`#include <common>
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
}`,Xm=`uniform vec3 color;
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
}`,qm=`uniform float rotation;
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
}`,Ym=`uniform vec3 diffuse;
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
}`,ke={alphahash_fragment:mf,alphahash_pars_fragment:gf,alphamap_fragment:_f,alphamap_pars_fragment:vf,alphatest_fragment:xf,alphatest_pars_fragment:yf,aomap_fragment:Sf,aomap_pars_fragment:Mf,batching_pars_vertex:bf,batching_vertex:Ef,begin_vertex:wf,beginnormal_vertex:Tf,bsdfs:Af,iridescence_fragment:Cf,bumpmap_pars_fragment:Rf,clipping_planes_fragment:Pf,clipping_planes_pars_fragment:Lf,clipping_planes_pars_vertex:Uf,clipping_planes_vertex:Df,color_fragment:If,color_pars_fragment:Nf,color_pars_vertex:Of,color_vertex:Ff,common:Bf,cube_uv_reflection_fragment:kf,defaultnormal_vertex:zf,displacementmap_pars_vertex:Vf,displacementmap_vertex:Gf,emissivemap_fragment:Hf,emissivemap_pars_fragment:Wf,colorspace_fragment:Xf,colorspace_pars_fragment:qf,envmap_fragment:Yf,envmap_common_pars_fragment:jf,envmap_pars_fragment:$f,envmap_pars_vertex:Kf,envmap_physical_pars_fragment:op,envmap_vertex:Zf,fog_vertex:Jf,fog_pars_vertex:Qf,fog_fragment:ep,fog_pars_fragment:tp,gradientmap_pars_fragment:np,lightmap_pars_fragment:ip,lights_lambert_fragment:rp,lights_lambert_pars_fragment:sp,lights_pars_begin:ap,lights_toon_fragment:lp,lights_toon_pars_fragment:cp,lights_phong_fragment:up,lights_phong_pars_fragment:hp,lights_physical_fragment:dp,lights_physical_pars_fragment:fp,lights_fragment_begin:pp,lights_fragment_maps:mp,lights_fragment_end:gp,logdepthbuf_fragment:_p,logdepthbuf_pars_fragment:vp,logdepthbuf_pars_vertex:xp,logdepthbuf_vertex:yp,map_fragment:Sp,map_pars_fragment:Mp,map_particle_fragment:bp,map_particle_pars_fragment:Ep,metalnessmap_fragment:wp,metalnessmap_pars_fragment:Tp,morphinstance_vertex:Ap,morphcolor_vertex:Cp,morphnormal_vertex:Rp,morphtarget_pars_vertex:Pp,morphtarget_vertex:Lp,normal_fragment_begin:Up,normal_fragment_maps:Dp,normal_pars_fragment:Ip,normal_pars_vertex:Np,normal_vertex:Op,normalmap_pars_fragment:Fp,clearcoat_normal_fragment_begin:Bp,clearcoat_normal_fragment_maps:kp,clearcoat_pars_fragment:zp,iridescence_pars_fragment:Vp,opaque_fragment:Gp,packing:Hp,premultiplied_alpha_fragment:Wp,project_vertex:Xp,dithering_fragment:qp,dithering_pars_fragment:Yp,roughnessmap_fragment:jp,roughnessmap_pars_fragment:$p,shadowmap_pars_fragment:Kp,shadowmap_pars_vertex:Zp,shadowmap_vertex:Jp,shadowmask_pars_fragment:Qp,skinbase_vertex:em,skinning_pars_vertex:tm,skinning_vertex:nm,skinnormal_vertex:im,specularmap_fragment:rm,specularmap_pars_fragment:sm,tonemapping_fragment:am,tonemapping_pars_fragment:om,transmission_fragment:lm,transmission_pars_fragment:cm,uv_pars_fragment:um,uv_pars_vertex:hm,uv_vertex:dm,worldpos_vertex:fm,background_vert:pm,background_frag:mm,backgroundCube_vert:gm,backgroundCube_frag:_m,cube_vert:vm,cube_frag:xm,depth_vert:ym,depth_frag:Sm,distanceRGBA_vert:Mm,distanceRGBA_frag:bm,equirect_vert:Em,equirect_frag:wm,linedashed_vert:Tm,linedashed_frag:Am,meshbasic_vert:Cm,meshbasic_frag:Rm,meshlambert_vert:Pm,meshlambert_frag:Lm,meshmatcap_vert:Um,meshmatcap_frag:Dm,meshnormal_vert:Im,meshnormal_frag:Nm,meshphong_vert:Om,meshphong_frag:Fm,meshphysical_vert:Bm,meshphysical_frag:km,meshtoon_vert:zm,meshtoon_frag:Vm,points_vert:Gm,points_frag:Hm,shadow_vert:Wm,shadow_frag:Xm,sprite_vert:qm,sprite_frag:Ym},fe={common:{diffuse:{value:new He(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new ze},alphaMap:{value:null},alphaMapTransform:{value:new ze},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new ze}},envmap:{envMap:{value:null},envMapRotation:{value:new ze},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new ze}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new ze}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new ze},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new ze},normalScale:{value:new De(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new ze},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new ze}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new ze}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new ze}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new He(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new He(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new ze},alphaTest:{value:0},uvTransform:{value:new ze}},sprite:{diffuse:{value:new He(16777215)},opacity:{value:1},center:{value:new De(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new ze},alphaMap:{value:null},alphaMapTransform:{value:new ze},alphaTest:{value:0}}},Bt={basic:{uniforms:_t([fe.common,fe.specularmap,fe.envmap,fe.aomap,fe.lightmap,fe.fog]),vertexShader:ke.meshbasic_vert,fragmentShader:ke.meshbasic_frag},lambert:{uniforms:_t([fe.common,fe.specularmap,fe.envmap,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.fog,fe.lights,{emissive:{value:new He(0)}}]),vertexShader:ke.meshlambert_vert,fragmentShader:ke.meshlambert_frag},phong:{uniforms:_t([fe.common,fe.specularmap,fe.envmap,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.fog,fe.lights,{emissive:{value:new He(0)},specular:{value:new He(1118481)},shininess:{value:30}}]),vertexShader:ke.meshphong_vert,fragmentShader:ke.meshphong_frag},standard:{uniforms:_t([fe.common,fe.envmap,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.roughnessmap,fe.metalnessmap,fe.fog,fe.lights,{emissive:{value:new He(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:ke.meshphysical_vert,fragmentShader:ke.meshphysical_frag},toon:{uniforms:_t([fe.common,fe.aomap,fe.lightmap,fe.emissivemap,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.gradientmap,fe.fog,fe.lights,{emissive:{value:new He(0)}}]),vertexShader:ke.meshtoon_vert,fragmentShader:ke.meshtoon_frag},matcap:{uniforms:_t([fe.common,fe.bumpmap,fe.normalmap,fe.displacementmap,fe.fog,{matcap:{value:null}}]),vertexShader:ke.meshmatcap_vert,fragmentShader:ke.meshmatcap_frag},points:{uniforms:_t([fe.points,fe.fog]),vertexShader:ke.points_vert,fragmentShader:ke.points_frag},dashed:{uniforms:_t([fe.common,fe.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:ke.linedashed_vert,fragmentShader:ke.linedashed_frag},depth:{uniforms:_t([fe.common,fe.displacementmap]),vertexShader:ke.depth_vert,fragmentShader:ke.depth_frag},normal:{uniforms:_t([fe.common,fe.bumpmap,fe.normalmap,fe.displacementmap,{opacity:{value:1}}]),vertexShader:ke.meshnormal_vert,fragmentShader:ke.meshnormal_frag},sprite:{uniforms:_t([fe.sprite,fe.fog]),vertexShader:ke.sprite_vert,fragmentShader:ke.sprite_frag},background:{uniforms:{uvTransform:{value:new ze},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:ke.background_vert,fragmentShader:ke.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new ze}},vertexShader:ke.backgroundCube_vert,fragmentShader:ke.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:ke.cube_vert,fragmentShader:ke.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:ke.equirect_vert,fragmentShader:ke.equirect_frag},distanceRGBA:{uniforms:_t([fe.common,fe.displacementmap,{referencePosition:{value:new G},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:ke.distanceRGBA_vert,fragmentShader:ke.distanceRGBA_frag},shadow:{uniforms:_t([fe.lights,fe.fog,{color:{value:new He(0)},opacity:{value:1}}]),vertexShader:ke.shadow_vert,fragmentShader:ke.shadow_frag}};Bt.physical={uniforms:_t([Bt.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new ze},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new ze},clearcoatNormalScale:{value:new De(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new ze},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new ze},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new ze},sheen:{value:0},sheenColor:{value:new He(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new ze},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new ze},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new ze},transmissionSamplerSize:{value:new De},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new ze},attenuationDistance:{value:0},attenuationColor:{value:new He(0)},specularColor:{value:new He(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new ze},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new ze},anisotropyVector:{value:new De},anisotropyMap:{value:null},anisotropyMapTransform:{value:new ze}}]),vertexShader:ke.meshphysical_vert,fragmentShader:ke.meshphysical_frag};const tr={r:0,b:0,g:0},wn=new Ht,jm=new Je;function $m(i,e,t,n,r,s,o){const a=new He(0);let l=s===!0?0:1,c,u,h=null,d=0,f=null;function g(w){let S=w.isScene===!0?w.background:null;return S&&S.isTexture&&(S=(w.backgroundBlurriness>0?t:e).get(S)),S}function x(w){let S=!1;const A=g(w);A===null?p(a,l):A&&A.isColor&&(p(A,1),S=!0);const k=i.xr.getEnvironmentBlendMode();k==="additive"?n.buffers.color.setClear(0,0,0,1,o):k==="alpha-blend"&&n.buffers.color.setClear(0,0,0,0,o),(i.autoClear||S)&&(n.buffers.depth.setTest(!0),n.buffers.depth.setMask(!0),n.buffers.color.setMask(!0),i.clear(i.autoClearColor,i.autoClearDepth,i.autoClearStencil))}function m(w,S){const A=g(S);A&&(A.isCubeTexture||A.mapping===Mr)?(u===void 0&&(u=new Ot(new Dn(1,1,1),new gn({name:"BackgroundCubeMaterial",uniforms:fi(Bt.backgroundCube.uniforms),vertexShader:Bt.backgroundCube.vertexShader,fragmentShader:Bt.backgroundCube.fragmentShader,side:yt,depthTest:!1,depthWrite:!1,fog:!1})),u.geometry.deleteAttribute("normal"),u.geometry.deleteAttribute("uv"),u.onBeforeRender=function(k,I,P){this.matrixWorld.copyPosition(P.matrixWorld)},Object.defineProperty(u.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),r.update(u)),wn.copy(S.backgroundRotation),wn.x*=-1,wn.y*=-1,wn.z*=-1,A.isCubeTexture&&A.isRenderTargetTexture===!1&&(wn.y*=-1,wn.z*=-1),u.material.uniforms.envMap.value=A,u.material.uniforms.flipEnvMap.value=A.isCubeTexture&&A.isRenderTargetTexture===!1?-1:1,u.material.uniforms.backgroundBlurriness.value=S.backgroundBlurriness,u.material.uniforms.backgroundIntensity.value=S.backgroundIntensity,u.material.uniforms.backgroundRotation.value.setFromMatrix4(jm.makeRotationFromEuler(wn)),u.material.toneMapped=je.getTransfer(A.colorSpace)!==Ke,(h!==A||d!==A.version||f!==i.toneMapping)&&(u.material.needsUpdate=!0,h=A,d=A.version,f=i.toneMapping),u.layers.enableAll(),w.unshift(u,u.geometry,u.material,0,0,null)):A&&A.isTexture&&(c===void 0&&(c=new Ot(new Tr(2,2),new gn({name:"BackgroundMaterial",uniforms:fi(Bt.background.uniforms),vertexShader:Bt.background.vertexShader,fragmentShader:Bt.background.fragmentShader,side:pn,depthTest:!1,depthWrite:!1,fog:!1})),c.geometry.deleteAttribute("normal"),Object.defineProperty(c.material,"map",{get:function(){return this.uniforms.t2D.value}}),r.update(c)),c.material.uniforms.t2D.value=A,c.material.uniforms.backgroundIntensity.value=S.backgroundIntensity,c.material.toneMapped=je.getTransfer(A.colorSpace)!==Ke,A.matrixAutoUpdate===!0&&A.updateMatrix(),c.material.uniforms.uvTransform.value.copy(A.matrix),(h!==A||d!==A.version||f!==i.toneMapping)&&(c.material.needsUpdate=!0,h=A,d=A.version,f=i.toneMapping),c.layers.enableAll(),w.unshift(c,c.geometry,c.material,0,0,null))}function p(w,S){w.getRGB(tr,Yc(i)),n.buffers.color.setClear(tr.r,tr.g,tr.b,S,o)}return{getClearColor:function(){return a},setClearColor:function(w,S=1){a.set(w),l=S,p(a,l)},getClearAlpha:function(){return l},setClearAlpha:function(w){l=w,p(a,l)},render:x,addToRenderList:m}}function Km(i,e){const t=i.getParameter(i.MAX_VERTEX_ATTRIBS),n={},r=d(null);let s=r,o=!1;function a(b,C,H,O,$){let z=!1;const q=h(O,H,C);s!==q&&(s=q,c(s.object)),z=f(b,O,H,$),z&&g(b,O,H,$),$!==null&&e.update($,i.ELEMENT_ARRAY_BUFFER),(z||o)&&(o=!1,A(b,C,H,O),$!==null&&i.bindBuffer(i.ELEMENT_ARRAY_BUFFER,e.get($).buffer))}function l(){return i.createVertexArray()}function c(b){return i.bindVertexArray(b)}function u(b){return i.deleteVertexArray(b)}function h(b,C,H){const O=H.wireframe===!0;let $=n[b.id];$===void 0&&($={},n[b.id]=$);let z=$[C.id];z===void 0&&(z={},$[C.id]=z);let q=z[O];return q===void 0&&(q=d(l()),z[O]=q),q}function d(b){const C=[],H=[],O=[];for(let $=0;$<t;$++)C[$]=0,H[$]=0,O[$]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:C,enabledAttributes:H,attributeDivisors:O,object:b,attributes:{},index:null}}function f(b,C,H,O){const $=s.attributes,z=C.attributes;let q=0;const te=H.getAttributes();for(const v in te)if(te[v].location>=0){const U=$[v];let N=z[v];if(N===void 0&&(v==="instanceMatrix"&&b.instanceMatrix&&(N=b.instanceMatrix),v==="instanceColor"&&b.instanceColor&&(N=b.instanceColor)),U===void 0||U.attribute!==N||N&&U.data!==N.data)return!0;q++}return s.attributesNum!==q||s.index!==O}function g(b,C,H,O){const $={},z=C.attributes;let q=0;const te=H.getAttributes();for(const v in te)if(te[v].location>=0){let U=z[v];U===void 0&&(v==="instanceMatrix"&&b.instanceMatrix&&(U=b.instanceMatrix),v==="instanceColor"&&b.instanceColor&&(U=b.instanceColor));const N={};N.attribute=U,U&&U.data&&(N.data=U.data),$[v]=N,q++}s.attributes=$,s.attributesNum=q,s.index=O}function x(){const b=s.newAttributes;for(let C=0,H=b.length;C<H;C++)b[C]=0}function m(b){p(b,0)}function p(b,C){const H=s.newAttributes,O=s.enabledAttributes,$=s.attributeDivisors;H[b]=1,O[b]===0&&(i.enableVertexAttribArray(b),O[b]=1),$[b]!==C&&(i.vertexAttribDivisor(b,C),$[b]=C)}function w(){const b=s.newAttributes,C=s.enabledAttributes;for(let H=0,O=C.length;H<O;H++)C[H]!==b[H]&&(i.disableVertexAttribArray(H),C[H]=0)}function S(b,C,H,O,$,z,q){q===!0?i.vertexAttribIPointer(b,C,H,$,z):i.vertexAttribPointer(b,C,H,O,$,z)}function A(b,C,H,O){x();const $=O.attributes,z=H.getAttributes(),q=C.defaultAttributeValues;for(const te in z){const v=z[te];if(v.location>=0){let T=$[te];if(T===void 0&&(te==="instanceMatrix"&&b.instanceMatrix&&(T=b.instanceMatrix),te==="instanceColor"&&b.instanceColor&&(T=b.instanceColor)),T!==void 0){const U=T.normalized,N=T.itemSize,V=e.get(T);if(V===void 0)continue;const K=V.buffer,D=V.type,B=V.bytesPerElement,j=D===i.INT||D===i.UNSIGNED_INT||T.gpuType===Pc;if(T.isInterleavedBufferAttribute){const X=T.data,le=X.stride,ve=T.offset;if(X.isInstancedInterleavedBuffer){for(let ge=0;ge<v.locationSize;ge++)p(v.location+ge,X.meshPerAttribute);b.isInstancedMesh!==!0&&O._maxInstanceCount===void 0&&(O._maxInstanceCount=X.meshPerAttribute*X.count)}else for(let ge=0;ge<v.locationSize;ge++)m(v.location+ge);i.bindBuffer(i.ARRAY_BUFFER,K);for(let ge=0;ge<v.locationSize;ge++)S(v.location+ge,N/v.locationSize,D,U,le*B,(ve+N/v.locationSize*ge)*B,j)}else{if(T.isInstancedBufferAttribute){for(let X=0;X<v.locationSize;X++)p(v.location+X,T.meshPerAttribute);b.isInstancedMesh!==!0&&O._maxInstanceCount===void 0&&(O._maxInstanceCount=T.meshPerAttribute*T.count)}else for(let X=0;X<v.locationSize;X++)m(v.location+X);i.bindBuffer(i.ARRAY_BUFFER,K);for(let X=0;X<v.locationSize;X++)S(v.location+X,N/v.locationSize,D,U,N*B,N/v.locationSize*X*B,j)}}else if(q!==void 0){const U=q[te];if(U!==void 0)switch(U.length){case 2:i.vertexAttrib2fv(v.location,U);break;case 3:i.vertexAttrib3fv(v.location,U);break;case 4:i.vertexAttrib4fv(v.location,U);break;default:i.vertexAttrib1fv(v.location,U)}}}}w()}function k(){Y();for(const b in n){const C=n[b];for(const H in C){const O=C[H];for(const $ in O)u(O[$].object),delete O[$];delete C[H]}delete n[b]}}function I(b){if(n[b.id]===void 0)return;const C=n[b.id];for(const H in C){const O=C[H];for(const $ in O)u(O[$].object),delete O[$];delete C[H]}delete n[b.id]}function P(b){for(const C in n){const H=n[C];if(H[b.id]===void 0)continue;const O=H[b.id];for(const $ in O)u(O[$].object),delete O[$];delete H[b.id]}}function Y(){E(),o=!0,s!==r&&(s=r,c(s.object))}function E(){r.geometry=null,r.program=null,r.wireframe=!1}return{setup:a,reset:Y,resetDefaultState:E,dispose:k,releaseStatesOfGeometry:I,releaseStatesOfProgram:P,initAttributes:x,enableAttribute:m,disableUnusedAttributes:w}}function Zm(i,e,t){let n;function r(c){n=c}function s(c,u){i.drawArrays(n,c,u),t.update(u,n,1)}function o(c,u,h){h!==0&&(i.drawArraysInstanced(n,c,u,h),t.update(u,n,h))}function a(c,u,h){if(h===0)return;const d=e.get("WEBGL_multi_draw");if(d===null)for(let f=0;f<h;f++)this.render(c[f],u[f]);else{d.multiDrawArraysWEBGL(n,c,0,u,0,h);let f=0;for(let g=0;g<h;g++)f+=u[g];t.update(f,n,1)}}function l(c,u,h,d){if(h===0)return;const f=e.get("WEBGL_multi_draw");if(f===null)for(let g=0;g<c.length;g++)o(c[g],u[g],d[g]);else{f.multiDrawArraysInstancedWEBGL(n,c,0,u,0,d,0,h);let g=0;for(let x=0;x<h;x++)g+=u[x];for(let x=0;x<d.length;x++)t.update(g,n,d[x])}}this.setMode=r,this.render=s,this.renderInstances=o,this.renderMultiDraw=a,this.renderMultiDrawInstances=l}function Jm(i,e,t,n){let r;function s(){if(r!==void 0)return r;if(e.has("EXT_texture_filter_anisotropic")===!0){const I=e.get("EXT_texture_filter_anisotropic");r=i.getParameter(I.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else r=0;return r}function o(I){return!(I!==zt&&n.convert(I)!==i.getParameter(i.IMPLEMENTATION_COLOR_READ_FORMAT))}function a(I){const P=I===br&&(e.has("EXT_color_buffer_half_float")||e.has("EXT_color_buffer_float"));return!(I!==mn&&n.convert(I)!==i.getParameter(i.IMPLEMENTATION_COLOR_READ_TYPE)&&I!==un&&!P)}function l(I){if(I==="highp"){if(i.getShaderPrecisionFormat(i.VERTEX_SHADER,i.HIGH_FLOAT).precision>0&&i.getShaderPrecisionFormat(i.FRAGMENT_SHADER,i.HIGH_FLOAT).precision>0)return"highp";I="mediump"}return I==="mediump"&&i.getShaderPrecisionFormat(i.VERTEX_SHADER,i.MEDIUM_FLOAT).precision>0&&i.getShaderPrecisionFormat(i.FRAGMENT_SHADER,i.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let c=t.precision!==void 0?t.precision:"highp";const u=l(c);u!==c&&(console.warn("THREE.WebGLRenderer:",c,"not supported, using",u,"instead."),c=u);const h=t.logarithmicDepthBuffer===!0,d=i.getParameter(i.MAX_TEXTURE_IMAGE_UNITS),f=i.getParameter(i.MAX_VERTEX_TEXTURE_IMAGE_UNITS),g=i.getParameter(i.MAX_TEXTURE_SIZE),x=i.getParameter(i.MAX_CUBE_MAP_TEXTURE_SIZE),m=i.getParameter(i.MAX_VERTEX_ATTRIBS),p=i.getParameter(i.MAX_VERTEX_UNIFORM_VECTORS),w=i.getParameter(i.MAX_VARYING_VECTORS),S=i.getParameter(i.MAX_FRAGMENT_UNIFORM_VECTORS),A=f>0,k=i.getParameter(i.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:s,getMaxPrecision:l,textureFormatReadable:o,textureTypeReadable:a,precision:c,logarithmicDepthBuffer:h,maxTextures:d,maxVertexTextures:f,maxTextureSize:g,maxCubemapSize:x,maxAttributes:m,maxVertexUniforms:p,maxVaryings:w,maxFragmentUniforms:S,vertexTextures:A,maxSamples:k}}function Qm(i){const e=this;let t=null,n=0,r=!1,s=!1;const o=new ln,a=new ze,l={value:null,needsUpdate:!1};this.uniform=l,this.numPlanes=0,this.numIntersection=0,this.init=function(h,d){const f=h.length!==0||d||n!==0||r;return r=d,n=h.length,f},this.beginShadows=function(){s=!0,u(null)},this.endShadows=function(){s=!1},this.setGlobalState=function(h,d){t=u(h,d,0)},this.setState=function(h,d,f){const g=h.clippingPlanes,x=h.clipIntersection,m=h.clipShadows,p=i.get(h);if(!r||g===null||g.length===0||s&&!m)s?u(null):c();else{const w=s?0:n,S=w*4;let A=p.clippingState||null;l.value=A,A=u(g,d,S,f);for(let k=0;k!==S;++k)A[k]=t[k];p.clippingState=A,this.numIntersection=x?this.numPlanes:0,this.numPlanes+=w}};function c(){l.value!==t&&(l.value=t,l.needsUpdate=n>0),e.numPlanes=n,e.numIntersection=0}function u(h,d,f,g){const x=h!==null?h.length:0;let m=null;if(x!==0){if(m=l.value,g!==!0||m===null){const p=f+x*4,w=d.matrixWorldInverse;a.getNormalMatrix(w),(m===null||m.length<p)&&(m=new Float32Array(p));for(let S=0,A=f;S!==x;++S,A+=4)o.copy(h[S]).applyMatrix4(w,a),o.normal.toArray(m,A),m[A+3]=o.constant}l.value=m,l.needsUpdate=!0}return e.numPlanes=x,e.numIntersection=0,m}}function eg(i){let e=new WeakMap;function t(o,a){return a===oa?o.mapping=li:a===la&&(o.mapping=ci),o}function n(o){if(o&&o.isTexture){const a=o.mapping;if(a===oa||a===la)if(e.has(o)){const l=e.get(o).texture;return t(l,o.mapping)}else{const l=o.image;if(l&&l.height>0){const c=new hf(l.height);return c.fromEquirectangularTexture(i,o),e.set(o,c),o.addEventListener("dispose",r),t(c.texture,o.mapping)}else return null}}return o}function r(o){const a=o.target;a.removeEventListener("dispose",r);const l=e.get(a);l!==void 0&&(e.delete(a),l.dispose())}function s(){e=new WeakMap}return{get:n,dispose:s}}class Zc extends jc{constructor(e=-1,t=1,n=1,r=-1,s=.1,o=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=e,this.right=t,this.top=n,this.bottom=r,this.near=s,this.far=o,this.updateProjectionMatrix()}copy(e,t){return super.copy(e,t),this.left=e.left,this.right=e.right,this.top=e.top,this.bottom=e.bottom,this.near=e.near,this.far=e.far,this.zoom=e.zoom,this.view=e.view===null?null:Object.assign({},e.view),this}setViewOffset(e,t,n,r,s,o){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=e,this.view.fullHeight=t,this.view.offsetX=n,this.view.offsetY=r,this.view.width=s,this.view.height=o,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const e=(this.right-this.left)/(2*this.zoom),t=(this.top-this.bottom)/(2*this.zoom),n=(this.right+this.left)/2,r=(this.top+this.bottom)/2;let s=n-e,o=n+e,a=r+t,l=r-t;if(this.view!==null&&this.view.enabled){const c=(this.right-this.left)/this.view.fullWidth/this.zoom,u=(this.top-this.bottom)/this.view.fullHeight/this.zoom;s+=c*this.view.offsetX,o=s+c*this.view.width,a-=u*this.view.offsetY,l=a-u*this.view.height}this.projectionMatrix.makeOrthographic(s,o,a,l,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(e){const t=super.toJSON(e);return t.object.zoom=this.zoom,t.object.left=this.left,t.object.right=this.right,t.object.top=this.top,t.object.bottom=this.bottom,t.object.near=this.near,t.object.far=this.far,this.view!==null&&(t.object.view=Object.assign({},this.view)),t}}const ri=4,El=[.125,.215,.35,.446,.526,.582],Rn=20,js=new Zc,wl=new He;let $s=null,Ks=0,Zs=0,Js=!1;const An=(1+Math.sqrt(5))/2,ii=1/An,Tl=[new G(-An,ii,0),new G(An,ii,0),new G(-ii,0,An),new G(ii,0,An),new G(0,An,-ii),new G(0,An,ii),new G(-1,1,-1),new G(1,1,-1),new G(-1,1,1),new G(1,1,1)];class Al{constructor(e){this._renderer=e,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(e,t=0,n=.1,r=100){$s=this._renderer.getRenderTarget(),Ks=this._renderer.getActiveCubeFace(),Zs=this._renderer.getActiveMipmapLevel(),Js=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(256);const s=this._allocateTargets();return s.depthBuffer=!0,this._sceneToCubeUV(e,n,r,s),t>0&&this._blur(s,0,0,t),this._applyPMREM(s),this._cleanup(s),s}fromEquirectangular(e,t=null){return this._fromTexture(e,t)}fromCubemap(e,t=null){return this._fromTexture(e,t)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Pl(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=Rl(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(e){this._lodMax=Math.floor(Math.log2(e)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let e=0;e<this._lodPlanes.length;e++)this._lodPlanes[e].dispose()}_cleanup(e){this._renderer.setRenderTarget($s,Ks,Zs),this._renderer.xr.enabled=Js,e.scissorTest=!1,nr(e,0,0,e.width,e.height)}_fromTexture(e,t){e.mapping===li||e.mapping===ci?this._setSize(e.image.length===0?16:e.image[0].width||e.image[0].image.width):this._setSize(e.image.width/4),$s=this._renderer.getRenderTarget(),Ks=this._renderer.getActiveCubeFace(),Zs=this._renderer.getActiveMipmapLevel(),Js=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;const n=t||this._allocateTargets();return this._textureToCubeUV(e,n),this._applyPMREM(n),this._cleanup(n),n}_allocateTargets(){const e=3*Math.max(this._cubeSize,112),t=4*this._cubeSize,n={magFilter:Nt,minFilter:Nt,generateMipmaps:!1,type:br,format:zt,colorSpace:_n,depthBuffer:!1},r=Cl(e,t,n);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==e||this._pingPongRenderTarget.height!==t){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=Cl(e,t,n);const{_lodMax:s}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=tg(s)),this._blurMaterial=ng(s,e,t)}return r}_compileMaterial(e){const t=new Ot(this._lodPlanes[0],e);this._renderer.compile(t,js)}_sceneToCubeUV(e,t,n,r){const a=new Rt(90,1,t,n),l=[1,-1,1,1,1,1],c=[1,1,1,-1,-1,-1],u=this._renderer,h=u.autoClear,d=u.toneMapping;u.getClearColor(wl),u.toneMapping=fn,u.autoClear=!1;const f=new Wc({name:"PMREM.Background",side:yt,depthWrite:!1,depthTest:!1}),g=new Ot(new Dn,f);let x=!1;const m=e.background;m?m.isColor&&(f.color.copy(m),e.background=null,x=!0):(f.color.copy(wl),x=!0);for(let p=0;p<6;p++){const w=p%3;w===0?(a.up.set(0,l[p],0),a.lookAt(c[p],0,0)):w===1?(a.up.set(0,0,l[p]),a.lookAt(0,c[p],0)):(a.up.set(0,l[p],0),a.lookAt(0,0,c[p]));const S=this._cubeSize;nr(r,w*S,p>2?S:0,S,S),u.setRenderTarget(r),x&&u.render(g,a),u.render(e,a)}g.geometry.dispose(),g.material.dispose(),u.toneMapping=d,u.autoClear=h,e.background=m}_textureToCubeUV(e,t){const n=this._renderer,r=e.mapping===li||e.mapping===ci;r?(this._cubemapMaterial===null&&(this._cubemapMaterial=Pl()),this._cubemapMaterial.uniforms.flipEnvMap.value=e.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=Rl());const s=r?this._cubemapMaterial:this._equirectMaterial,o=new Ot(this._lodPlanes[0],s),a=s.uniforms;a.envMap.value=e;const l=this._cubeSize;nr(t,0,0,3*l,2*l),n.setRenderTarget(t),n.render(o,js)}_applyPMREM(e){const t=this._renderer,n=t.autoClear;t.autoClear=!1;const r=this._lodPlanes.length;for(let s=1;s<r;s++){const o=Math.sqrt(this._sigmas[s]*this._sigmas[s]-this._sigmas[s-1]*this._sigmas[s-1]),a=Tl[(r-s-1)%Tl.length];this._blur(e,s-1,s,o,a)}t.autoClear=n}_blur(e,t,n,r,s){const o=this._pingPongRenderTarget;this._halfBlur(e,o,t,n,r,"latitudinal",s),this._halfBlur(o,e,n,n,r,"longitudinal",s)}_halfBlur(e,t,n,r,s,o,a){const l=this._renderer,c=this._blurMaterial;o!=="latitudinal"&&o!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const u=3,h=new Ot(this._lodPlanes[r],c),d=c.uniforms,f=this._sizeLods[n]-1,g=isFinite(s)?Math.PI/(2*f):2*Math.PI/(2*Rn-1),x=s/g,m=isFinite(s)?1+Math.floor(u*x):Rn;m>Rn&&console.warn(`sigmaRadians, ${s}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${Rn}`);const p=[];let w=0;for(let P=0;P<Rn;++P){const Y=P/x,E=Math.exp(-Y*Y/2);p.push(E),P===0?w+=E:P<m&&(w+=2*E)}for(let P=0;P<p.length;P++)p[P]=p[P]/w;d.envMap.value=e.texture,d.samples.value=m,d.weights.value=p,d.latitudinal.value=o==="latitudinal",a&&(d.poleAxis.value=a);const{_lodMax:S}=this;d.dTheta.value=g,d.mipInt.value=S-n;const A=this._sizeLods[r],k=3*A*(r>S-ri?r-S+ri:0),I=4*(this._cubeSize-A);nr(t,k,I,3*A,2*A),l.setRenderTarget(t),l.render(h,js)}}function tg(i){const e=[],t=[],n=[];let r=i;const s=i-ri+1+El.length;for(let o=0;o<s;o++){const a=Math.pow(2,r);t.push(a);let l=1/a;o>i-ri?l=El[o-i+ri-1]:o===0&&(l=0),n.push(l);const c=1/(a-2),u=-c,h=1+c,d=[u,u,h,u,h,h,u,u,h,h,u,h],f=6,g=6,x=3,m=2,p=1,w=new Float32Array(x*g*f),S=new Float32Array(m*g*f),A=new Float32Array(p*g*f);for(let I=0;I<f;I++){const P=I%3*2/3-1,Y=I>2?0:-1,E=[P,Y,0,P+2/3,Y,0,P+2/3,Y+1,0,P,Y,0,P+2/3,Y+1,0,P,Y+1,0];w.set(E,x*g*I),S.set(d,m*g*I);const b=[I,I,I,I,I,I];A.set(b,p*g*I)}const k=new Lt;k.setAttribute("position",new Vt(w,x)),k.setAttribute("uv",new Vt(S,m)),k.setAttribute("faceIndex",new Vt(A,p)),e.push(k),r>ri&&r--}return{lodPlanes:e,sizeLods:t,sigmas:n}}function Cl(i,e,t){const n=new Un(i,e,t);return n.texture.mapping=Mr,n.texture.name="PMREM.cubeUv",n.scissorTest=!0,n}function nr(i,e,t,n,r){i.viewport.set(e,t,n,r),i.scissor.set(e,t,n,r)}function ng(i,e,t){const n=new Float32Array(Rn),r=new G(0,1,0);return new gn({name:"SphericalGaussianBlur",defines:{n:Rn,CUBEUV_TEXEL_WIDTH:1/e,CUBEUV_TEXEL_HEIGHT:1/t,CUBEUV_MAX_MIP:`${i}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:n},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:r}},vertexShader:Ma(),fragmentShader:`

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
		`,blending:dn,depthTest:!1,depthWrite:!1})}function Rl(){return new gn({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:Ma(),fragmentShader:`

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
		`,blending:dn,depthTest:!1,depthWrite:!1})}function Pl(){return new gn({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:Ma(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:dn,depthTest:!1,depthWrite:!1})}function Ma(){return`

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
	`}function ig(i){let e=new WeakMap,t=null;function n(a){if(a&&a.isTexture){const l=a.mapping,c=l===oa||l===la,u=l===li||l===ci;if(c||u){let h=e.get(a);const d=h!==void 0?h.texture.pmremVersion:0;if(a.isRenderTargetTexture&&a.pmremVersion!==d)return t===null&&(t=new Al(i)),h=c?t.fromEquirectangular(a,h):t.fromCubemap(a,h),h.texture.pmremVersion=a.pmremVersion,e.set(a,h),h.texture;if(h!==void 0)return h.texture;{const f=a.image;return c&&f&&f.height>0||u&&f&&r(f)?(t===null&&(t=new Al(i)),h=c?t.fromEquirectangular(a):t.fromCubemap(a),h.texture.pmremVersion=a.pmremVersion,e.set(a,h),a.addEventListener("dispose",s),h.texture):null}}}return a}function r(a){let l=0;const c=6;for(let u=0;u<c;u++)a[u]!==void 0&&l++;return l===c}function s(a){const l=a.target;l.removeEventListener("dispose",s);const c=e.get(l);c!==void 0&&(e.delete(l),c.dispose())}function o(){e=new WeakMap,t!==null&&(t.dispose(),t=null)}return{get:n,dispose:o}}function rg(i){const e={};function t(n){if(e[n]!==void 0)return e[n];let r;switch(n){case"WEBGL_depth_texture":r=i.getExtension("WEBGL_depth_texture")||i.getExtension("MOZ_WEBGL_depth_texture")||i.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":r=i.getExtension("EXT_texture_filter_anisotropic")||i.getExtension("MOZ_EXT_texture_filter_anisotropic")||i.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":r=i.getExtension("WEBGL_compressed_texture_s3tc")||i.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||i.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":r=i.getExtension("WEBGL_compressed_texture_pvrtc")||i.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:r=i.getExtension(n)}return e[n]=r,r}return{has:function(n){return t(n)!==null},init:function(){t("EXT_color_buffer_float"),t("WEBGL_clip_cull_distance"),t("OES_texture_float_linear"),t("EXT_color_buffer_half_float"),t("WEBGL_multisampled_render_to_texture"),t("WEBGL_render_shared_exponent")},get:function(n){const r=t(n);return r===null&&kc("THREE.WebGLRenderer: "+n+" extension not supported."),r}}}function sg(i,e,t,n){const r={},s=new WeakMap;function o(h){const d=h.target;d.index!==null&&e.remove(d.index);for(const g in d.attributes)e.remove(d.attributes[g]);for(const g in d.morphAttributes){const x=d.morphAttributes[g];for(let m=0,p=x.length;m<p;m++)e.remove(x[m])}d.removeEventListener("dispose",o),delete r[d.id];const f=s.get(d);f&&(e.remove(f),s.delete(d)),n.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,t.memory.geometries--}function a(h,d){return r[d.id]===!0||(d.addEventListener("dispose",o),r[d.id]=!0,t.memory.geometries++),d}function l(h){const d=h.attributes;for(const g in d)e.update(d[g],i.ARRAY_BUFFER);const f=h.morphAttributes;for(const g in f){const x=f[g];for(let m=0,p=x.length;m<p;m++)e.update(x[m],i.ARRAY_BUFFER)}}function c(h){const d=[],f=h.index,g=h.attributes.position;let x=0;if(f!==null){const w=f.array;x=f.version;for(let S=0,A=w.length;S<A;S+=3){const k=w[S+0],I=w[S+1],P=w[S+2];d.push(k,I,I,P,P,k)}}else if(g!==void 0){const w=g.array;x=g.version;for(let S=0,A=w.length/3-1;S<A;S+=3){const k=S+0,I=S+1,P=S+2;d.push(k,I,I,P,P,k)}}else return;const m=new(Bc(d)?qc:Xc)(d,1);m.version=x;const p=s.get(h);p&&e.remove(p),s.set(h,m)}function u(h){const d=s.get(h);if(d){const f=h.index;f!==null&&d.version<f.version&&c(h)}else c(h);return s.get(h)}return{get:a,update:l,getWireframeAttribute:u}}function ag(i,e,t){let n;function r(d){n=d}let s,o;function a(d){s=d.type,o=d.bytesPerElement}function l(d,f){i.drawElements(n,f,s,d*o),t.update(f,n,1)}function c(d,f,g){g!==0&&(i.drawElementsInstanced(n,f,s,d*o,g),t.update(f,n,g))}function u(d,f,g){if(g===0)return;const x=e.get("WEBGL_multi_draw");if(x===null)for(let m=0;m<g;m++)this.render(d[m]/o,f[m]);else{x.multiDrawElementsWEBGL(n,f,0,s,d,0,g);let m=0;for(let p=0;p<g;p++)m+=f[p];t.update(m,n,1)}}function h(d,f,g,x){if(g===0)return;const m=e.get("WEBGL_multi_draw");if(m===null)for(let p=0;p<d.length;p++)c(d[p]/o,f[p],x[p]);else{m.multiDrawElementsInstancedWEBGL(n,f,0,s,d,0,x,0,g);let p=0;for(let w=0;w<g;w++)p+=f[w];for(let w=0;w<x.length;w++)t.update(p,n,x[w])}}this.setMode=r,this.setIndex=a,this.render=l,this.renderInstances=c,this.renderMultiDraw=u,this.renderMultiDrawInstances=h}function og(i){const e={geometries:0,textures:0},t={frame:0,calls:0,triangles:0,points:0,lines:0};function n(s,o,a){switch(t.calls++,o){case i.TRIANGLES:t.triangles+=a*(s/3);break;case i.LINES:t.lines+=a*(s/2);break;case i.LINE_STRIP:t.lines+=a*(s-1);break;case i.LINE_LOOP:t.lines+=a*s;break;case i.POINTS:t.points+=a*s;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",o);break}}function r(){t.calls=0,t.triangles=0,t.points=0,t.lines=0}return{memory:e,render:t,programs:null,autoReset:!0,reset:r,update:n}}function lg(i,e,t){const n=new WeakMap,r=new ht;function s(o,a,l){const c=o.morphTargetInfluences,u=a.morphAttributes.position||a.morphAttributes.normal||a.morphAttributes.color,h=u!==void 0?u.length:0;let d=n.get(a);if(d===void 0||d.count!==h){let b=function(){Y.dispose(),n.delete(a),a.removeEventListener("dispose",b)};var f=b;d!==void 0&&d.texture.dispose();const g=a.morphAttributes.position!==void 0,x=a.morphAttributes.normal!==void 0,m=a.morphAttributes.color!==void 0,p=a.morphAttributes.position||[],w=a.morphAttributes.normal||[],S=a.morphAttributes.color||[];let A=0;g===!0&&(A=1),x===!0&&(A=2),m===!0&&(A=3);let k=a.attributes.position.count*A,I=1;k>e.maxTextureSize&&(I=Math.ceil(k/e.maxTextureSize),k=e.maxTextureSize);const P=new Float32Array(k*I*4*h),Y=new Vc(P,k,I,h);Y.type=un,Y.needsUpdate=!0;const E=A*4;for(let C=0;C<h;C++){const H=p[C],O=w[C],$=S[C],z=k*I*4*C;for(let q=0;q<H.count;q++){const te=q*E;g===!0&&(r.fromBufferAttribute(H,q),P[z+te+0]=r.x,P[z+te+1]=r.y,P[z+te+2]=r.z,P[z+te+3]=0),x===!0&&(r.fromBufferAttribute(O,q),P[z+te+4]=r.x,P[z+te+5]=r.y,P[z+te+6]=r.z,P[z+te+7]=0),m===!0&&(r.fromBufferAttribute($,q),P[z+te+8]=r.x,P[z+te+9]=r.y,P[z+te+10]=r.z,P[z+te+11]=$.itemSize===4?r.w:1)}}d={count:h,texture:Y,size:new De(k,I)},n.set(a,d),a.addEventListener("dispose",b)}if(o.isInstancedMesh===!0&&o.morphTexture!==null)l.getUniforms().setValue(i,"morphTexture",o.morphTexture,t);else{let g=0;for(let m=0;m<c.length;m++)g+=c[m];const x=a.morphTargetsRelative?1:1-g;l.getUniforms().setValue(i,"morphTargetBaseInfluence",x),l.getUniforms().setValue(i,"morphTargetInfluences",c)}l.getUniforms().setValue(i,"morphTargetsTexture",d.texture,t),l.getUniforms().setValue(i,"morphTargetsTextureSize",d.size)}return{update:s}}function cg(i,e,t,n){let r=new WeakMap;function s(l){const c=n.render.frame,u=l.geometry,h=e.get(l,u);if(r.get(h)!==c&&(e.update(h),r.set(h,c)),l.isInstancedMesh&&(l.hasEventListener("dispose",a)===!1&&l.addEventListener("dispose",a),r.get(l)!==c&&(t.update(l.instanceMatrix,i.ARRAY_BUFFER),l.instanceColor!==null&&t.update(l.instanceColor,i.ARRAY_BUFFER),r.set(l,c))),l.isSkinnedMesh){const d=l.skeleton;r.get(d)!==c&&(d.update(),r.set(d,c))}return h}function o(){r=new WeakMap}function a(l){const c=l.target;c.removeEventListener("dispose",a),t.remove(c.instanceMatrix),c.instanceColor!==null&&t.remove(c.instanceColor)}return{update:s,dispose:o}}class Jc extends St{constructor(e,t,n,r,s,o,a,l,c,u=ai){if(u!==ai&&u!==di)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");n===void 0&&u===ai&&(n=ui),n===void 0&&u===di&&(n=hi),super(null,r,s,o,a,l,u,n,c),this.isDepthTexture=!0,this.image={width:e,height:t},this.magFilter=a!==void 0?a:Pt,this.minFilter=l!==void 0?l:Pt,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(e){return super.copy(e),this.compareFunction=e.compareFunction,this}toJSON(e){const t=super.toJSON(e);return this.compareFunction!==null&&(t.compareFunction=this.compareFunction),t}}const Qc=new St,eu=new Jc(1,1);eu.compareFunction=Fc;const tu=new Vc,nu=new jd,iu=new $c,Ll=[],Ul=[],Dl=new Float32Array(16),Il=new Float32Array(9),Nl=new Float32Array(4);function gi(i,e,t){const n=i[0];if(n<=0||n>0)return i;const r=e*t;let s=Ll[r];if(s===void 0&&(s=new Float32Array(r),Ll[r]=s),e!==0){n.toArray(s,0);for(let o=1,a=0;o!==e;++o)a+=t,i[o].toArray(s,a)}return s}function ot(i,e){if(i.length!==e.length)return!1;for(let t=0,n=i.length;t<n;t++)if(i[t]!==e[t])return!1;return!0}function lt(i,e){for(let t=0,n=e.length;t<n;t++)i[t]=e[t]}function Ar(i,e){let t=Ul[e];t===void 0&&(t=new Int32Array(e),Ul[e]=t);for(let n=0;n!==e;++n)t[n]=i.allocateTextureUnit();return t}function ug(i,e){const t=this.cache;t[0]!==e&&(i.uniform1f(this.addr,e),t[0]=e)}function hg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(i.uniform2f(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(ot(t,e))return;i.uniform2fv(this.addr,e),lt(t,e)}}function dg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(i.uniform3f(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else if(e.r!==void 0)(t[0]!==e.r||t[1]!==e.g||t[2]!==e.b)&&(i.uniform3f(this.addr,e.r,e.g,e.b),t[0]=e.r,t[1]=e.g,t[2]=e.b);else{if(ot(t,e))return;i.uniform3fv(this.addr,e),lt(t,e)}}function fg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(i.uniform4f(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(ot(t,e))return;i.uniform4fv(this.addr,e),lt(t,e)}}function pg(i,e){const t=this.cache,n=e.elements;if(n===void 0){if(ot(t,e))return;i.uniformMatrix2fv(this.addr,!1,e),lt(t,e)}else{if(ot(t,n))return;Nl.set(n),i.uniformMatrix2fv(this.addr,!1,Nl),lt(t,n)}}function mg(i,e){const t=this.cache,n=e.elements;if(n===void 0){if(ot(t,e))return;i.uniformMatrix3fv(this.addr,!1,e),lt(t,e)}else{if(ot(t,n))return;Il.set(n),i.uniformMatrix3fv(this.addr,!1,Il),lt(t,n)}}function gg(i,e){const t=this.cache,n=e.elements;if(n===void 0){if(ot(t,e))return;i.uniformMatrix4fv(this.addr,!1,e),lt(t,e)}else{if(ot(t,n))return;Dl.set(n),i.uniformMatrix4fv(this.addr,!1,Dl),lt(t,n)}}function _g(i,e){const t=this.cache;t[0]!==e&&(i.uniform1i(this.addr,e),t[0]=e)}function vg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(i.uniform2i(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(ot(t,e))return;i.uniform2iv(this.addr,e),lt(t,e)}}function xg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(i.uniform3i(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(ot(t,e))return;i.uniform3iv(this.addr,e),lt(t,e)}}function yg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(i.uniform4i(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(ot(t,e))return;i.uniform4iv(this.addr,e),lt(t,e)}}function Sg(i,e){const t=this.cache;t[0]!==e&&(i.uniform1ui(this.addr,e),t[0]=e)}function Mg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y)&&(i.uniform2ui(this.addr,e.x,e.y),t[0]=e.x,t[1]=e.y);else{if(ot(t,e))return;i.uniform2uiv(this.addr,e),lt(t,e)}}function bg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z)&&(i.uniform3ui(this.addr,e.x,e.y,e.z),t[0]=e.x,t[1]=e.y,t[2]=e.z);else{if(ot(t,e))return;i.uniform3uiv(this.addr,e),lt(t,e)}}function Eg(i,e){const t=this.cache;if(e.x!==void 0)(t[0]!==e.x||t[1]!==e.y||t[2]!==e.z||t[3]!==e.w)&&(i.uniform4ui(this.addr,e.x,e.y,e.z,e.w),t[0]=e.x,t[1]=e.y,t[2]=e.z,t[3]=e.w);else{if(ot(t,e))return;i.uniform4uiv(this.addr,e),lt(t,e)}}function wg(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r);const s=this.type===i.SAMPLER_2D_SHADOW?eu:Qc;t.setTexture2D(e||s,r)}function Tg(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r),t.setTexture3D(e||nu,r)}function Ag(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r),t.setTextureCube(e||iu,r)}function Cg(i,e,t){const n=this.cache,r=t.allocateTextureUnit();n[0]!==r&&(i.uniform1i(this.addr,r),n[0]=r),t.setTexture2DArray(e||tu,r)}function Rg(i){switch(i){case 5126:return ug;case 35664:return hg;case 35665:return dg;case 35666:return fg;case 35674:return pg;case 35675:return mg;case 35676:return gg;case 5124:case 35670:return _g;case 35667:case 35671:return vg;case 35668:case 35672:return xg;case 35669:case 35673:return yg;case 5125:return Sg;case 36294:return Mg;case 36295:return bg;case 36296:return Eg;case 35678:case 36198:case 36298:case 36306:case 35682:return wg;case 35679:case 36299:case 36307:return Tg;case 35680:case 36300:case 36308:case 36293:return Ag;case 36289:case 36303:case 36311:case 36292:return Cg}}function Pg(i,e){i.uniform1fv(this.addr,e)}function Lg(i,e){const t=gi(e,this.size,2);i.uniform2fv(this.addr,t)}function Ug(i,e){const t=gi(e,this.size,3);i.uniform3fv(this.addr,t)}function Dg(i,e){const t=gi(e,this.size,4);i.uniform4fv(this.addr,t)}function Ig(i,e){const t=gi(e,this.size,4);i.uniformMatrix2fv(this.addr,!1,t)}function Ng(i,e){const t=gi(e,this.size,9);i.uniformMatrix3fv(this.addr,!1,t)}function Og(i,e){const t=gi(e,this.size,16);i.uniformMatrix4fv(this.addr,!1,t)}function Fg(i,e){i.uniform1iv(this.addr,e)}function Bg(i,e){i.uniform2iv(this.addr,e)}function kg(i,e){i.uniform3iv(this.addr,e)}function zg(i,e){i.uniform4iv(this.addr,e)}function Vg(i,e){i.uniform1uiv(this.addr,e)}function Gg(i,e){i.uniform2uiv(this.addr,e)}function Hg(i,e){i.uniform3uiv(this.addr,e)}function Wg(i,e){i.uniform4uiv(this.addr,e)}function Xg(i,e,t){const n=this.cache,r=e.length,s=Ar(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTexture2D(e[o]||Qc,s[o])}function qg(i,e,t){const n=this.cache,r=e.length,s=Ar(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTexture3D(e[o]||nu,s[o])}function Yg(i,e,t){const n=this.cache,r=e.length,s=Ar(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTextureCube(e[o]||iu,s[o])}function jg(i,e,t){const n=this.cache,r=e.length,s=Ar(t,r);ot(n,s)||(i.uniform1iv(this.addr,s),lt(n,s));for(let o=0;o!==r;++o)t.setTexture2DArray(e[o]||tu,s[o])}function $g(i){switch(i){case 5126:return Pg;case 35664:return Lg;case 35665:return Ug;case 35666:return Dg;case 35674:return Ig;case 35675:return Ng;case 35676:return Og;case 5124:case 35670:return Fg;case 35667:case 35671:return Bg;case 35668:case 35672:return kg;case 35669:case 35673:return zg;case 5125:return Vg;case 36294:return Gg;case 36295:return Hg;case 36296:return Wg;case 35678:case 36198:case 36298:case 36306:case 35682:return Xg;case 35679:case 36299:case 36307:return qg;case 35680:case 36300:case 36308:case 36293:return Yg;case 36289:case 36303:case 36311:case 36292:return jg}}class Kg{constructor(e,t,n){this.id=e,this.addr=n,this.cache=[],this.type=t.type,this.setValue=Rg(t.type)}}class Zg{constructor(e,t,n){this.id=e,this.addr=n,this.cache=[],this.type=t.type,this.size=t.size,this.setValue=$g(t.type)}}class Jg{constructor(e){this.id=e,this.seq=[],this.map={}}setValue(e,t,n){const r=this.seq;for(let s=0,o=r.length;s!==o;++s){const a=r[s];a.setValue(e,t[a.id],n)}}}const Qs=/(\w+)(\])?(\[|\.)?/g;function Ol(i,e){i.seq.push(e),i.map[e.id]=e}function Qg(i,e,t){const n=i.name,r=n.length;for(Qs.lastIndex=0;;){const s=Qs.exec(n),o=Qs.lastIndex;let a=s[1];const l=s[2]==="]",c=s[3];if(l&&(a=a|0),c===void 0||c==="["&&o+2===r){Ol(t,c===void 0?new Kg(a,i,e):new Zg(a,i,e));break}else{let h=t.map[a];h===void 0&&(h=new Jg(a),Ol(t,h)),t=h}}}class hr{constructor(e,t){this.seq=[],this.map={};const n=e.getProgramParameter(t,e.ACTIVE_UNIFORMS);for(let r=0;r<n;++r){const s=e.getActiveUniform(t,r),o=e.getUniformLocation(t,s.name);Qg(s,o,this)}}setValue(e,t,n,r){const s=this.map[t];s!==void 0&&s.setValue(e,n,r)}setOptional(e,t,n){const r=t[n];r!==void 0&&this.setValue(e,n,r)}static upload(e,t,n,r){for(let s=0,o=t.length;s!==o;++s){const a=t[s],l=n[a.id];l.needsUpdate!==!1&&a.setValue(e,l.value,r)}}static seqWithValue(e,t){const n=[];for(let r=0,s=e.length;r!==s;++r){const o=e[r];o.id in t&&n.push(o)}return n}}function Fl(i,e,t){const n=i.createShader(e);return i.shaderSource(n,t),i.compileShader(n),n}const e_=37297;let t_=0;function n_(i,e){const t=i.split(`
`),n=[],r=Math.max(e-6,0),s=Math.min(e+6,t.length);for(let o=r;o<s;o++){const a=o+1;n.push(`${a===e?">":" "} ${a}: ${t[o]}`)}return n.join(`
`)}function i_(i){const e=je.getPrimaries(je.workingColorSpace),t=je.getPrimaries(i);let n;switch(e===t?n="":e===gr&&t===mr?n="LinearDisplayP3ToLinearSRGB":e===mr&&t===gr&&(n="LinearSRGBToLinearDisplayP3"),i){case _n:case Er:return[n,"LinearTransferOETF"];case Ft:case ya:return[n,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space:",i),[n,"LinearTransferOETF"]}}function Bl(i,e,t){const n=i.getShaderParameter(e,i.COMPILE_STATUS),r=i.getShaderInfoLog(e).trim();if(n&&r==="")return"";const s=/ERROR: 0:(\d+)/.exec(r);if(s){const o=parseInt(s[1]);return t.toUpperCase()+`

`+r+`

`+n_(i.getShaderSource(e),o)}else return r}function r_(i,e){const t=i_(e);return`vec4 ${i}( vec4 value ) { return ${t[0]}( ${t[1]}( value ) ); }`}function s_(i,e){let t;switch(e){case hd:t="Linear";break;case dd:t="Reinhard";break;case fd:t="OptimizedCineon";break;case pd:t="ACESFilmic";break;case gd:t="AgX";break;case _d:t="Neutral";break;case md:t="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",e),t="Linear"}return"vec3 "+i+"( vec3 color ) { return "+t+"ToneMapping( color ); }"}function a_(i){return[i.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",i.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(Ei).join(`
`)}function o_(i){const e=[];for(const t in i){const n=i[t];n!==!1&&e.push("#define "+t+" "+n)}return e.join(`
`)}function l_(i,e){const t={},n=i.getProgramParameter(e,i.ACTIVE_ATTRIBUTES);for(let r=0;r<n;r++){const s=i.getActiveAttrib(e,r),o=s.name;let a=1;s.type===i.FLOAT_MAT2&&(a=2),s.type===i.FLOAT_MAT3&&(a=3),s.type===i.FLOAT_MAT4&&(a=4),t[o]={type:s.type,location:i.getAttribLocation(e,o),locationSize:a}}return t}function Ei(i){return i!==""}function kl(i,e){const t=e.numSpotLightShadows+e.numSpotLightMaps-e.numSpotLightShadowsWithMaps;return i.replace(/NUM_DIR_LIGHTS/g,e.numDirLights).replace(/NUM_SPOT_LIGHTS/g,e.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,e.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,t).replace(/NUM_RECT_AREA_LIGHTS/g,e.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,e.numPointLights).replace(/NUM_HEMI_LIGHTS/g,e.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,e.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,e.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,e.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,e.numPointLightShadows)}function zl(i,e){return i.replace(/NUM_CLIPPING_PLANES/g,e.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,e.numClippingPlanes-e.numClipIntersection)}const c_=/^[ \t]*#include +<([\w\d./]+)>/gm;function da(i){return i.replace(c_,h_)}const u_=new Map;function h_(i,e){let t=ke[e];if(t===void 0){const n=u_.get(e);if(n!==void 0)t=ke[n],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',e,n);else throw new Error("Can not resolve #include <"+e+">")}return da(t)}const d_=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function Vl(i){return i.replace(d_,f_)}function f_(i,e,t,n){let r="";for(let s=parseInt(e);s<parseInt(t);s++)r+=n.replace(/\[\s*i\s*\]/g,"[ "+s+" ]").replace(/UNROLLED_LOOP_INDEX/g,s);return r}function Gl(i){let e=`precision ${i.precision} float;
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
#define LOW_PRECISION`),e}function p_(i){let e="SHADOWMAP_TYPE_BASIC";return i.shadowMapType===Ac?e="SHADOWMAP_TYPE_PCF":i.shadowMapType===Fh?e="SHADOWMAP_TYPE_PCF_SOFT":i.shadowMapType===Kt&&(e="SHADOWMAP_TYPE_VSM"),e}function m_(i){let e="ENVMAP_TYPE_CUBE";if(i.envMap)switch(i.envMapMode){case li:case ci:e="ENVMAP_TYPE_CUBE";break;case Mr:e="ENVMAP_TYPE_CUBE_UV";break}return e}function g_(i){let e="ENVMAP_MODE_REFLECTION";if(i.envMap)switch(i.envMapMode){case ci:e="ENVMAP_MODE_REFRACTION";break}return e}function __(i){let e="ENVMAP_BLENDING_NONE";if(i.envMap)switch(i.combine){case Cc:e="ENVMAP_BLENDING_MULTIPLY";break;case cd:e="ENVMAP_BLENDING_MIX";break;case ud:e="ENVMAP_BLENDING_ADD";break}return e}function v_(i){const e=i.envMapCubeUVHeight;if(e===null)return null;const t=Math.log2(e)-2,n=1/e;return{texelWidth:1/(3*Math.max(Math.pow(2,t),7*16)),texelHeight:n,maxMip:t}}function x_(i,e,t,n){const r=i.getContext(),s=t.defines;let o=t.vertexShader,a=t.fragmentShader;const l=p_(t),c=m_(t),u=g_(t),h=__(t),d=v_(t),f=a_(t),g=o_(s),x=r.createProgram();let m,p,w=t.glslVersion?"#version "+t.glslVersion+`
`:"";t.isRawShaderMaterial?(m=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g].filter(Ei).join(`
`),m.length>0&&(m+=`
`),p=["#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g].filter(Ei).join(`
`),p.length>0&&(p+=`
`)):(m=[Gl(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g,t.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",t.batching?"#define USE_BATCHING":"",t.batchingColor?"#define USE_BATCHING_COLOR":"",t.instancing?"#define USE_INSTANCING":"",t.instancingColor?"#define USE_INSTANCING_COLOR":"",t.instancingMorph?"#define USE_INSTANCING_MORPH":"",t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.map?"#define USE_MAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+u:"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.displacementMap?"#define USE_DISPLACEMENTMAP":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.mapUv?"#define MAP_UV "+t.mapUv:"",t.alphaMapUv?"#define ALPHAMAP_UV "+t.alphaMapUv:"",t.lightMapUv?"#define LIGHTMAP_UV "+t.lightMapUv:"",t.aoMapUv?"#define AOMAP_UV "+t.aoMapUv:"",t.emissiveMapUv?"#define EMISSIVEMAP_UV "+t.emissiveMapUv:"",t.bumpMapUv?"#define BUMPMAP_UV "+t.bumpMapUv:"",t.normalMapUv?"#define NORMALMAP_UV "+t.normalMapUv:"",t.displacementMapUv?"#define DISPLACEMENTMAP_UV "+t.displacementMapUv:"",t.metalnessMapUv?"#define METALNESSMAP_UV "+t.metalnessMapUv:"",t.roughnessMapUv?"#define ROUGHNESSMAP_UV "+t.roughnessMapUv:"",t.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+t.anisotropyMapUv:"",t.clearcoatMapUv?"#define CLEARCOATMAP_UV "+t.clearcoatMapUv:"",t.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+t.clearcoatNormalMapUv:"",t.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+t.clearcoatRoughnessMapUv:"",t.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+t.iridescenceMapUv:"",t.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+t.iridescenceThicknessMapUv:"",t.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+t.sheenColorMapUv:"",t.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+t.sheenRoughnessMapUv:"",t.specularMapUv?"#define SPECULARMAP_UV "+t.specularMapUv:"",t.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+t.specularColorMapUv:"",t.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+t.specularIntensityMapUv:"",t.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+t.transmissionMapUv:"",t.thicknessMapUv?"#define THICKNESSMAP_UV "+t.thicknessMapUv:"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.flatShading?"#define FLAT_SHADED":"",t.skinning?"#define USE_SKINNING":"",t.morphTargets?"#define USE_MORPHTARGETS":"",t.morphNormals&&t.flatShading===!1?"#define USE_MORPHNORMALS":"",t.morphColors?"#define USE_MORPHCOLORS":"",t.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+t.morphTextureStride:"",t.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+t.morphTargetsCount:"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+l:"",t.sizeAttenuation?"#define USE_SIZEATTENUATION":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(Ei).join(`
`),p=[Gl(t),"#define SHADER_TYPE "+t.shaderType,"#define SHADER_NAME "+t.shaderName,g,t.useFog&&t.fog?"#define USE_FOG":"",t.useFog&&t.fogExp2?"#define FOG_EXP2":"",t.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",t.map?"#define USE_MAP":"",t.matcap?"#define USE_MATCAP":"",t.envMap?"#define USE_ENVMAP":"",t.envMap?"#define "+c:"",t.envMap?"#define "+u:"",t.envMap?"#define "+h:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",t.lightMap?"#define USE_LIGHTMAP":"",t.aoMap?"#define USE_AOMAP":"",t.bumpMap?"#define USE_BUMPMAP":"",t.normalMap?"#define USE_NORMALMAP":"",t.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",t.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",t.emissiveMap?"#define USE_EMISSIVEMAP":"",t.anisotropy?"#define USE_ANISOTROPY":"",t.anisotropyMap?"#define USE_ANISOTROPYMAP":"",t.clearcoat?"#define USE_CLEARCOAT":"",t.clearcoatMap?"#define USE_CLEARCOATMAP":"",t.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",t.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",t.dispersion?"#define USE_DISPERSION":"",t.iridescence?"#define USE_IRIDESCENCE":"",t.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",t.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",t.specularMap?"#define USE_SPECULARMAP":"",t.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",t.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",t.roughnessMap?"#define USE_ROUGHNESSMAP":"",t.metalnessMap?"#define USE_METALNESSMAP":"",t.alphaMap?"#define USE_ALPHAMAP":"",t.alphaTest?"#define USE_ALPHATEST":"",t.alphaHash?"#define USE_ALPHAHASH":"",t.sheen?"#define USE_SHEEN":"",t.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",t.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",t.transmission?"#define USE_TRANSMISSION":"",t.transmissionMap?"#define USE_TRANSMISSIONMAP":"",t.thicknessMap?"#define USE_THICKNESSMAP":"",t.vertexTangents&&t.flatShading===!1?"#define USE_TANGENT":"",t.vertexColors||t.instancingColor||t.batchingColor?"#define USE_COLOR":"",t.vertexAlphas?"#define USE_COLOR_ALPHA":"",t.vertexUv1s?"#define USE_UV1":"",t.vertexUv2s?"#define USE_UV2":"",t.vertexUv3s?"#define USE_UV3":"",t.pointsUvs?"#define USE_POINTS_UV":"",t.gradientMap?"#define USE_GRADIENTMAP":"",t.flatShading?"#define FLAT_SHADED":"",t.doubleSided?"#define DOUBLE_SIDED":"",t.flipSided?"#define FLIP_SIDED":"",t.shadowMapEnabled?"#define USE_SHADOWMAP":"",t.shadowMapEnabled?"#define "+l:"",t.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",t.numLightProbes>0?"#define USE_LIGHT_PROBES":"",t.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",t.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",t.toneMapping!==fn?"#define TONE_MAPPING":"",t.toneMapping!==fn?ke.tonemapping_pars_fragment:"",t.toneMapping!==fn?s_("toneMapping",t.toneMapping):"",t.dithering?"#define DITHERING":"",t.opaque?"#define OPAQUE":"",ke.colorspace_pars_fragment,r_("linearToOutputTexel",t.outputColorSpace),t.useDepthPacking?"#define DEPTH_PACKING "+t.depthPacking:"",`
`].filter(Ei).join(`
`)),o=da(o),o=kl(o,t),o=zl(o,t),a=da(a),a=kl(a,t),a=zl(a,t),o=Vl(o),a=Vl(a),t.isRawShaderMaterial!==!0&&(w=`#version 300 es
`,m=[f,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+m,p=["#define varying in",t.glslVersion===rl?"":"layout(location = 0) out highp vec4 pc_fragColor;",t.glslVersion===rl?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+p);const S=w+m+o,A=w+p+a,k=Fl(r,r.VERTEX_SHADER,S),I=Fl(r,r.FRAGMENT_SHADER,A);r.attachShader(x,k),r.attachShader(x,I),t.index0AttributeName!==void 0?r.bindAttribLocation(x,0,t.index0AttributeName):t.morphTargets===!0&&r.bindAttribLocation(x,0,"position"),r.linkProgram(x);function P(C){if(i.debug.checkShaderErrors){const H=r.getProgramInfoLog(x).trim(),O=r.getShaderInfoLog(k).trim(),$=r.getShaderInfoLog(I).trim();let z=!0,q=!0;if(r.getProgramParameter(x,r.LINK_STATUS)===!1)if(z=!1,typeof i.debug.onShaderError=="function")i.debug.onShaderError(r,x,k,I);else{const te=Bl(r,k,"vertex"),v=Bl(r,I,"fragment");console.error("THREE.WebGLProgram: Shader Error "+r.getError()+" - VALIDATE_STATUS "+r.getProgramParameter(x,r.VALIDATE_STATUS)+`

Material Name: `+C.name+`
Material Type: `+C.type+`

Program Info Log: `+H+`
`+te+`
`+v)}else H!==""?console.warn("THREE.WebGLProgram: Program Info Log:",H):(O===""||$==="")&&(q=!1);q&&(C.diagnostics={runnable:z,programLog:H,vertexShader:{log:O,prefix:m},fragmentShader:{log:$,prefix:p}})}r.deleteShader(k),r.deleteShader(I),Y=new hr(r,x),E=l_(r,x)}let Y;this.getUniforms=function(){return Y===void 0&&P(this),Y};let E;this.getAttributes=function(){return E===void 0&&P(this),E};let b=t.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return b===!1&&(b=r.getProgramParameter(x,e_)),b},this.destroy=function(){n.releaseStatesOfProgram(this),r.deleteProgram(x),this.program=void 0},this.type=t.shaderType,this.name=t.shaderName,this.id=t_++,this.cacheKey=e,this.usedTimes=1,this.program=x,this.vertexShader=k,this.fragmentShader=I,this}let y_=0;class S_{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(e){const t=e.vertexShader,n=e.fragmentShader,r=this._getShaderStage(t),s=this._getShaderStage(n),o=this._getShaderCacheForMaterial(e);return o.has(r)===!1&&(o.add(r),r.usedTimes++),o.has(s)===!1&&(o.add(s),s.usedTimes++),this}remove(e){const t=this.materialCache.get(e);for(const n of t)n.usedTimes--,n.usedTimes===0&&this.shaderCache.delete(n.code);return this.materialCache.delete(e),this}getVertexShaderID(e){return this._getShaderStage(e.vertexShader).id}getFragmentShaderID(e){return this._getShaderStage(e.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(e){const t=this.materialCache;let n=t.get(e);return n===void 0&&(n=new Set,t.set(e,n)),n}_getShaderStage(e){const t=this.shaderCache;let n=t.get(e);return n===void 0&&(n=new M_(e),t.set(e,n)),n}}class M_{constructor(e){this.id=y_++,this.code=e,this.usedTimes=0}}function b_(i,e,t,n,r,s,o){const a=new Gc,l=new S_,c=new Set,u=[],h=r.logarithmicDepthBuffer,d=r.vertexTextures;let f=r.precision;const g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function x(E){return c.add(E),E===0?"uv":`uv${E}`}function m(E,b,C,H,O){const $=H.fog,z=O.geometry,q=E.isMeshStandardMaterial?H.environment:null,te=(E.isMeshStandardMaterial?t:e).get(E.envMap||q),v=te&&te.mapping===Mr?te.image.height:null,T=g[E.type];E.precision!==null&&(f=r.getMaxPrecision(E.precision),f!==E.precision&&console.warn("THREE.WebGLProgram.getParameters:",E.precision,"not supported, using",f,"instead."));const U=z.morphAttributes.position||z.morphAttributes.normal||z.morphAttributes.color,N=U!==void 0?U.length:0;let V=0;z.morphAttributes.position!==void 0&&(V=1),z.morphAttributes.normal!==void 0&&(V=2),z.morphAttributes.color!==void 0&&(V=3);let K,D,B,j;if(T){const We=Bt[T];K=We.vertexShader,D=We.fragmentShader}else K=E.vertexShader,D=E.fragmentShader,l.update(E),B=l.getVertexShaderID(E),j=l.getFragmentShaderID(E);const X=i.getRenderTarget(),le=O.isInstancedMesh===!0,ve=O.isBatchedMesh===!0,ge=!!E.map,L=!!E.matcap,xe=!!te,we=!!E.aoMap,Xe=!!E.lightMap,Me=!!E.bumpMap,Be=!!E.normalMap,Ie=!!E.displacementMap,Ae=!!E.emissiveMap,Ve=!!E.metalnessMap,R=!!E.roughnessMap,y=E.anisotropy>0,ee=E.clearcoat>0,re=E.dispersion>0,ae=E.iridescence>0,oe=E.sheen>0,be=E.transmission>0,de=y&&!!E.anisotropyMap,he=ee&&!!E.clearcoatMap,Ne=ee&&!!E.clearcoatNormalMap,ce=ee&&!!E.clearcoatRoughnessMap,ye=ae&&!!E.iridescenceMap,Ge=ae&&!!E.iridescenceThicknessMap,Re=oe&&!!E.sheenColorMap,pe=oe&&!!E.sheenRoughnessMap,Oe=!!E.specularMap,Fe=!!E.specularColorMap,Qe=!!E.specularIntensityMap,_=be&&!!E.transmissionMap,ne=be&&!!E.thicknessMap,Z=!!E.gradientMap,ie=!!E.alphaMap,se=E.alphaTest>0,Ee=!!E.alphaHash,Ue=!!E.extensions;let et=fn;E.toneMapped&&(X===null||X.isXRRenderTarget===!0)&&(et=i.toneMapping);const it={shaderID:T,shaderType:E.type,shaderName:E.name,vertexShader:K,fragmentShader:D,defines:E.defines,customVertexShaderID:B,customFragmentShaderID:j,isRawShaderMaterial:E.isRawShaderMaterial===!0,glslVersion:E.glslVersion,precision:f,batching:ve,batchingColor:ve&&O._colorsTexture!==null,instancing:le,instancingColor:le&&O.instanceColor!==null,instancingMorph:le&&O.morphTexture!==null,supportsVertexTextures:d,outputColorSpace:X===null?i.outputColorSpace:X.isXRRenderTarget===!0?X.texture.colorSpace:_n,alphaToCoverage:!!E.alphaToCoverage,map:ge,matcap:L,envMap:xe,envMapMode:xe&&te.mapping,envMapCubeUVHeight:v,aoMap:we,lightMap:Xe,bumpMap:Me,normalMap:Be,displacementMap:d&&Ie,emissiveMap:Ae,normalMapObjectSpace:Be&&E.normalMapType===Ld,normalMapTangentSpace:Be&&E.normalMapType===Oc,metalnessMap:Ve,roughnessMap:R,anisotropy:y,anisotropyMap:de,clearcoat:ee,clearcoatMap:he,clearcoatNormalMap:Ne,clearcoatRoughnessMap:ce,dispersion:re,iridescence:ae,iridescenceMap:ye,iridescenceThicknessMap:Ge,sheen:oe,sheenColorMap:Re,sheenRoughnessMap:pe,specularMap:Oe,specularColorMap:Fe,specularIntensityMap:Qe,transmission:be,transmissionMap:_,thicknessMap:ne,gradientMap:Z,opaque:E.transparent===!1&&E.blending===si&&E.alphaToCoverage===!1,alphaMap:ie,alphaTest:se,alphaHash:Ee,combine:E.combine,mapUv:ge&&x(E.map.channel),aoMapUv:we&&x(E.aoMap.channel),lightMapUv:Xe&&x(E.lightMap.channel),bumpMapUv:Me&&x(E.bumpMap.channel),normalMapUv:Be&&x(E.normalMap.channel),displacementMapUv:Ie&&x(E.displacementMap.channel),emissiveMapUv:Ae&&x(E.emissiveMap.channel),metalnessMapUv:Ve&&x(E.metalnessMap.channel),roughnessMapUv:R&&x(E.roughnessMap.channel),anisotropyMapUv:de&&x(E.anisotropyMap.channel),clearcoatMapUv:he&&x(E.clearcoatMap.channel),clearcoatNormalMapUv:Ne&&x(E.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:ce&&x(E.clearcoatRoughnessMap.channel),iridescenceMapUv:ye&&x(E.iridescenceMap.channel),iridescenceThicknessMapUv:Ge&&x(E.iridescenceThicknessMap.channel),sheenColorMapUv:Re&&x(E.sheenColorMap.channel),sheenRoughnessMapUv:pe&&x(E.sheenRoughnessMap.channel),specularMapUv:Oe&&x(E.specularMap.channel),specularColorMapUv:Fe&&x(E.specularColorMap.channel),specularIntensityMapUv:Qe&&x(E.specularIntensityMap.channel),transmissionMapUv:_&&x(E.transmissionMap.channel),thicknessMapUv:ne&&x(E.thicknessMap.channel),alphaMapUv:ie&&x(E.alphaMap.channel),vertexTangents:!!z.attributes.tangent&&(Be||y),vertexColors:E.vertexColors,vertexAlphas:E.vertexColors===!0&&!!z.attributes.color&&z.attributes.color.itemSize===4,pointsUvs:O.isPoints===!0&&!!z.attributes.uv&&(ge||ie),fog:!!$,useFog:E.fog===!0,fogExp2:!!$&&$.isFogExp2,flatShading:E.flatShading===!0,sizeAttenuation:E.sizeAttenuation===!0,logarithmicDepthBuffer:h,skinning:O.isSkinnedMesh===!0,morphTargets:z.morphAttributes.position!==void 0,morphNormals:z.morphAttributes.normal!==void 0,morphColors:z.morphAttributes.color!==void 0,morphTargetsCount:N,morphTextureStride:V,numDirLights:b.directional.length,numPointLights:b.point.length,numSpotLights:b.spot.length,numSpotLightMaps:b.spotLightMap.length,numRectAreaLights:b.rectArea.length,numHemiLights:b.hemi.length,numDirLightShadows:b.directionalShadowMap.length,numPointLightShadows:b.pointShadowMap.length,numSpotLightShadows:b.spotShadowMap.length,numSpotLightShadowsWithMaps:b.numSpotLightShadowsWithMaps,numLightProbes:b.numLightProbes,numClippingPlanes:o.numPlanes,numClipIntersection:o.numIntersection,dithering:E.dithering,shadowMapEnabled:i.shadowMap.enabled&&C.length>0,shadowMapType:i.shadowMap.type,toneMapping:et,decodeVideoTexture:ge&&E.map.isVideoTexture===!0&&je.getTransfer(E.map.colorSpace)===Ke,premultipliedAlpha:E.premultipliedAlpha,doubleSided:E.side===Zt,flipSided:E.side===yt,useDepthPacking:E.depthPacking>=0,depthPacking:E.depthPacking||0,index0AttributeName:E.index0AttributeName,extensionClipCullDistance:Ue&&E.extensions.clipCullDistance===!0&&n.has("WEBGL_clip_cull_distance"),extensionMultiDraw:Ue&&E.extensions.multiDraw===!0&&n.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:n.has("KHR_parallel_shader_compile"),customProgramCacheKey:E.customProgramCacheKey()};return it.vertexUv1s=c.has(1),it.vertexUv2s=c.has(2),it.vertexUv3s=c.has(3),c.clear(),it}function p(E){const b=[];if(E.shaderID?b.push(E.shaderID):(b.push(E.customVertexShaderID),b.push(E.customFragmentShaderID)),E.defines!==void 0)for(const C in E.defines)b.push(C),b.push(E.defines[C]);return E.isRawShaderMaterial===!1&&(w(b,E),S(b,E),b.push(i.outputColorSpace)),b.push(E.customProgramCacheKey),b.join()}function w(E,b){E.push(b.precision),E.push(b.outputColorSpace),E.push(b.envMapMode),E.push(b.envMapCubeUVHeight),E.push(b.mapUv),E.push(b.alphaMapUv),E.push(b.lightMapUv),E.push(b.aoMapUv),E.push(b.bumpMapUv),E.push(b.normalMapUv),E.push(b.displacementMapUv),E.push(b.emissiveMapUv),E.push(b.metalnessMapUv),E.push(b.roughnessMapUv),E.push(b.anisotropyMapUv),E.push(b.clearcoatMapUv),E.push(b.clearcoatNormalMapUv),E.push(b.clearcoatRoughnessMapUv),E.push(b.iridescenceMapUv),E.push(b.iridescenceThicknessMapUv),E.push(b.sheenColorMapUv),E.push(b.sheenRoughnessMapUv),E.push(b.specularMapUv),E.push(b.specularColorMapUv),E.push(b.specularIntensityMapUv),E.push(b.transmissionMapUv),E.push(b.thicknessMapUv),E.push(b.combine),E.push(b.fogExp2),E.push(b.sizeAttenuation),E.push(b.morphTargetsCount),E.push(b.morphAttributeCount),E.push(b.numDirLights),E.push(b.numPointLights),E.push(b.numSpotLights),E.push(b.numSpotLightMaps),E.push(b.numHemiLights),E.push(b.numRectAreaLights),E.push(b.numDirLightShadows),E.push(b.numPointLightShadows),E.push(b.numSpotLightShadows),E.push(b.numSpotLightShadowsWithMaps),E.push(b.numLightProbes),E.push(b.shadowMapType),E.push(b.toneMapping),E.push(b.numClippingPlanes),E.push(b.numClipIntersection),E.push(b.depthPacking)}function S(E,b){a.disableAll(),b.supportsVertexTextures&&a.enable(0),b.instancing&&a.enable(1),b.instancingColor&&a.enable(2),b.instancingMorph&&a.enable(3),b.matcap&&a.enable(4),b.envMap&&a.enable(5),b.normalMapObjectSpace&&a.enable(6),b.normalMapTangentSpace&&a.enable(7),b.clearcoat&&a.enable(8),b.iridescence&&a.enable(9),b.alphaTest&&a.enable(10),b.vertexColors&&a.enable(11),b.vertexAlphas&&a.enable(12),b.vertexUv1s&&a.enable(13),b.vertexUv2s&&a.enable(14),b.vertexUv3s&&a.enable(15),b.vertexTangents&&a.enable(16),b.anisotropy&&a.enable(17),b.alphaHash&&a.enable(18),b.batching&&a.enable(19),b.dispersion&&a.enable(20),b.batchingColor&&a.enable(21),E.push(a.mask),a.disableAll(),b.fog&&a.enable(0),b.useFog&&a.enable(1),b.flatShading&&a.enable(2),b.logarithmicDepthBuffer&&a.enable(3),b.skinning&&a.enable(4),b.morphTargets&&a.enable(5),b.morphNormals&&a.enable(6),b.morphColors&&a.enable(7),b.premultipliedAlpha&&a.enable(8),b.shadowMapEnabled&&a.enable(9),b.doubleSided&&a.enable(10),b.flipSided&&a.enable(11),b.useDepthPacking&&a.enable(12),b.dithering&&a.enable(13),b.transmission&&a.enable(14),b.sheen&&a.enable(15),b.opaque&&a.enable(16),b.pointsUvs&&a.enable(17),b.decodeVideoTexture&&a.enable(18),b.alphaToCoverage&&a.enable(19),E.push(a.mask)}function A(E){const b=g[E.type];let C;if(b){const H=Bt[b];C=of.clone(H.uniforms)}else C=E.uniforms;return C}function k(E,b){let C;for(let H=0,O=u.length;H<O;H++){const $=u[H];if($.cacheKey===b){C=$,++C.usedTimes;break}}return C===void 0&&(C=new x_(i,b,E,s),u.push(C)),C}function I(E){if(--E.usedTimes===0){const b=u.indexOf(E);u[b]=u[u.length-1],u.pop(),E.destroy()}}function P(E){l.remove(E)}function Y(){l.dispose()}return{getParameters:m,getProgramCacheKey:p,getUniforms:A,acquireProgram:k,releaseProgram:I,releaseShaderCache:P,programs:u,dispose:Y}}function E_(){let i=new WeakMap;function e(s){let o=i.get(s);return o===void 0&&(o={},i.set(s,o)),o}function t(s){i.delete(s)}function n(s,o,a){i.get(s)[o]=a}function r(){i=new WeakMap}return{get:e,remove:t,update:n,dispose:r}}function w_(i,e){return i.groupOrder!==e.groupOrder?i.groupOrder-e.groupOrder:i.renderOrder!==e.renderOrder?i.renderOrder-e.renderOrder:i.material.id!==e.material.id?i.material.id-e.material.id:i.z!==e.z?i.z-e.z:i.id-e.id}function Hl(i,e){return i.groupOrder!==e.groupOrder?i.groupOrder-e.groupOrder:i.renderOrder!==e.renderOrder?i.renderOrder-e.renderOrder:i.z!==e.z?e.z-i.z:i.id-e.id}function Wl(){const i=[];let e=0;const t=[],n=[],r=[];function s(){e=0,t.length=0,n.length=0,r.length=0}function o(h,d,f,g,x,m){let p=i[e];return p===void 0?(p={id:h.id,object:h,geometry:d,material:f,groupOrder:g,renderOrder:h.renderOrder,z:x,group:m},i[e]=p):(p.id=h.id,p.object=h,p.geometry=d,p.material=f,p.groupOrder=g,p.renderOrder=h.renderOrder,p.z=x,p.group=m),e++,p}function a(h,d,f,g,x,m){const p=o(h,d,f,g,x,m);f.transmission>0?n.push(p):f.transparent===!0?r.push(p):t.push(p)}function l(h,d,f,g,x,m){const p=o(h,d,f,g,x,m);f.transmission>0?n.unshift(p):f.transparent===!0?r.unshift(p):t.unshift(p)}function c(h,d){t.length>1&&t.sort(h||w_),n.length>1&&n.sort(d||Hl),r.length>1&&r.sort(d||Hl)}function u(){for(let h=e,d=i.length;h<d;h++){const f=i[h];if(f.id===null)break;f.id=null,f.object=null,f.geometry=null,f.material=null,f.group=null}}return{opaque:t,transmissive:n,transparent:r,init:s,push:a,unshift:l,finish:u,sort:c}}function T_(){let i=new WeakMap;function e(n,r){const s=i.get(n);let o;return s===void 0?(o=new Wl,i.set(n,[o])):r>=s.length?(o=new Wl,s.push(o)):o=s[r],o}function t(){i=new WeakMap}return{get:e,dispose:t}}function A_(){const i={};return{get:function(e){if(i[e.id]!==void 0)return i[e.id];let t;switch(e.type){case"DirectionalLight":t={direction:new G,color:new He};break;case"SpotLight":t={position:new G,direction:new G,color:new He,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":t={position:new G,color:new He,distance:0,decay:0};break;case"HemisphereLight":t={direction:new G,skyColor:new He,groundColor:new He};break;case"RectAreaLight":t={color:new He,position:new G,halfWidth:new G,halfHeight:new G};break}return i[e.id]=t,t}}}function C_(){const i={};return{get:function(e){if(i[e.id]!==void 0)return i[e.id];let t;switch(e.type){case"DirectionalLight":t={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new De};break;case"SpotLight":t={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new De};break;case"PointLight":t={shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new De,shadowCameraNear:1,shadowCameraFar:1e3};break}return i[e.id]=t,t}}}let R_=0;function P_(i,e){return(e.castShadow?2:0)-(i.castShadow?2:0)+(e.map?1:0)-(i.map?1:0)}function L_(i){const e=new A_,t=C_(),n={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let c=0;c<9;c++)n.probe.push(new G);const r=new G,s=new Je,o=new Je;function a(c){let u=0,h=0,d=0;for(let E=0;E<9;E++)n.probe[E].set(0,0,0);let f=0,g=0,x=0,m=0,p=0,w=0,S=0,A=0,k=0,I=0,P=0;c.sort(P_);for(let E=0,b=c.length;E<b;E++){const C=c[E],H=C.color,O=C.intensity,$=C.distance,z=C.shadow&&C.shadow.map?C.shadow.map.texture:null;if(C.isAmbientLight)u+=H.r*O,h+=H.g*O,d+=H.b*O;else if(C.isLightProbe){for(let q=0;q<9;q++)n.probe[q].addScaledVector(C.sh.coefficients[q],O);P++}else if(C.isDirectionalLight){const q=e.get(C);if(q.color.copy(C.color).multiplyScalar(C.intensity),C.castShadow){const te=C.shadow,v=t.get(C);v.shadowBias=te.bias,v.shadowNormalBias=te.normalBias,v.shadowRadius=te.radius,v.shadowMapSize=te.mapSize,n.directionalShadow[f]=v,n.directionalShadowMap[f]=z,n.directionalShadowMatrix[f]=C.shadow.matrix,w++}n.directional[f]=q,f++}else if(C.isSpotLight){const q=e.get(C);q.position.setFromMatrixPosition(C.matrixWorld),q.color.copy(H).multiplyScalar(O),q.distance=$,q.coneCos=Math.cos(C.angle),q.penumbraCos=Math.cos(C.angle*(1-C.penumbra)),q.decay=C.decay,n.spot[x]=q;const te=C.shadow;if(C.map&&(n.spotLightMap[k]=C.map,k++,te.updateMatrices(C),C.castShadow&&I++),n.spotLightMatrix[x]=te.matrix,C.castShadow){const v=t.get(C);v.shadowBias=te.bias,v.shadowNormalBias=te.normalBias,v.shadowRadius=te.radius,v.shadowMapSize=te.mapSize,n.spotShadow[x]=v,n.spotShadowMap[x]=z,A++}x++}else if(C.isRectAreaLight){const q=e.get(C);q.color.copy(H).multiplyScalar(O),q.halfWidth.set(C.width*.5,0,0),q.halfHeight.set(0,C.height*.5,0),n.rectArea[m]=q,m++}else if(C.isPointLight){const q=e.get(C);if(q.color.copy(C.color).multiplyScalar(C.intensity),q.distance=C.distance,q.decay=C.decay,C.castShadow){const te=C.shadow,v=t.get(C);v.shadowBias=te.bias,v.shadowNormalBias=te.normalBias,v.shadowRadius=te.radius,v.shadowMapSize=te.mapSize,v.shadowCameraNear=te.camera.near,v.shadowCameraFar=te.camera.far,n.pointShadow[g]=v,n.pointShadowMap[g]=z,n.pointShadowMatrix[g]=C.shadow.matrix,S++}n.point[g]=q,g++}else if(C.isHemisphereLight){const q=e.get(C);q.skyColor.copy(C.color).multiplyScalar(O),q.groundColor.copy(C.groundColor).multiplyScalar(O),n.hemi[p]=q,p++}}m>0&&(i.has("OES_texture_float_linear")===!0?(n.rectAreaLTC1=fe.LTC_FLOAT_1,n.rectAreaLTC2=fe.LTC_FLOAT_2):(n.rectAreaLTC1=fe.LTC_HALF_1,n.rectAreaLTC2=fe.LTC_HALF_2)),n.ambient[0]=u,n.ambient[1]=h,n.ambient[2]=d;const Y=n.hash;(Y.directionalLength!==f||Y.pointLength!==g||Y.spotLength!==x||Y.rectAreaLength!==m||Y.hemiLength!==p||Y.numDirectionalShadows!==w||Y.numPointShadows!==S||Y.numSpotShadows!==A||Y.numSpotMaps!==k||Y.numLightProbes!==P)&&(n.directional.length=f,n.spot.length=x,n.rectArea.length=m,n.point.length=g,n.hemi.length=p,n.directionalShadow.length=w,n.directionalShadowMap.length=w,n.pointShadow.length=S,n.pointShadowMap.length=S,n.spotShadow.length=A,n.spotShadowMap.length=A,n.directionalShadowMatrix.length=w,n.pointShadowMatrix.length=S,n.spotLightMatrix.length=A+k-I,n.spotLightMap.length=k,n.numSpotLightShadowsWithMaps=I,n.numLightProbes=P,Y.directionalLength=f,Y.pointLength=g,Y.spotLength=x,Y.rectAreaLength=m,Y.hemiLength=p,Y.numDirectionalShadows=w,Y.numPointShadows=S,Y.numSpotShadows=A,Y.numSpotMaps=k,Y.numLightProbes=P,n.version=R_++)}function l(c,u){let h=0,d=0,f=0,g=0,x=0;const m=u.matrixWorldInverse;for(let p=0,w=c.length;p<w;p++){const S=c[p];if(S.isDirectionalLight){const A=n.directional[h];A.direction.setFromMatrixPosition(S.matrixWorld),r.setFromMatrixPosition(S.target.matrixWorld),A.direction.sub(r),A.direction.transformDirection(m),h++}else if(S.isSpotLight){const A=n.spot[f];A.position.setFromMatrixPosition(S.matrixWorld),A.position.applyMatrix4(m),A.direction.setFromMatrixPosition(S.matrixWorld),r.setFromMatrixPosition(S.target.matrixWorld),A.direction.sub(r),A.direction.transformDirection(m),f++}else if(S.isRectAreaLight){const A=n.rectArea[g];A.position.setFromMatrixPosition(S.matrixWorld),A.position.applyMatrix4(m),o.identity(),s.copy(S.matrixWorld),s.premultiply(m),o.extractRotation(s),A.halfWidth.set(S.width*.5,0,0),A.halfHeight.set(0,S.height*.5,0),A.halfWidth.applyMatrix4(o),A.halfHeight.applyMatrix4(o),g++}else if(S.isPointLight){const A=n.point[d];A.position.setFromMatrixPosition(S.matrixWorld),A.position.applyMatrix4(m),d++}else if(S.isHemisphereLight){const A=n.hemi[x];A.direction.setFromMatrixPosition(S.matrixWorld),A.direction.transformDirection(m),x++}}}return{setup:a,setupView:l,state:n}}function Xl(i){const e=new L_(i),t=[],n=[];function r(u){c.camera=u,t.length=0,n.length=0}function s(u){t.push(u)}function o(u){n.push(u)}function a(){e.setup(t)}function l(u){e.setupView(t,u)}const c={lightsArray:t,shadowsArray:n,camera:null,lights:e,transmissionRenderTarget:{}};return{init:r,state:c,setupLights:a,setupLightsView:l,pushLight:s,pushShadow:o}}function U_(i){let e=new WeakMap;function t(r,s=0){const o=e.get(r);let a;return o===void 0?(a=new Xl(i),e.set(r,[a])):s>=o.length?(a=new Xl(i),o.push(a)):a=o[s],a}function n(){e=new WeakMap}return{get:t,dispose:n}}class D_ extends Bn{constructor(e){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=Rd,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(e)}copy(e){return super.copy(e),this.depthPacking=e.depthPacking,this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this}}class I_ extends Bn{constructor(e){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(e)}copy(e){return super.copy(e),this.map=e.map,this.alphaMap=e.alphaMap,this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this}}const N_=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,O_=`uniform sampler2D shadow_pass;
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
}`;function F_(i,e,t){let n=new Sa;const r=new De,s=new De,o=new ht,a=new D_({depthPacking:Pd}),l=new I_,c={},u=t.maxTextureSize,h={[pn]:yt,[yt]:pn,[Zt]:Zt},d=new gn({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new De},radius:{value:4}},vertexShader:N_,fragmentShader:O_}),f=d.clone();f.defines.HORIZONTAL_PASS=1;const g=new Lt;g.setAttribute("position",new Vt(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const x=new Ot(g,d),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=Ac;let p=this.type;this.render=function(I,P,Y){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||I.length===0)return;const E=i.getRenderTarget(),b=i.getActiveCubeFace(),C=i.getActiveMipmapLevel(),H=i.state;H.setBlending(dn),H.buffers.color.setClear(1,1,1,1),H.buffers.depth.setTest(!0),H.setScissorTest(!1);const O=p!==Kt&&this.type===Kt,$=p===Kt&&this.type!==Kt;for(let z=0,q=I.length;z<q;z++){const te=I[z],v=te.shadow;if(v===void 0){console.warn("THREE.WebGLShadowMap:",te,"has no shadow.");continue}if(v.autoUpdate===!1&&v.needsUpdate===!1)continue;r.copy(v.mapSize);const T=v.getFrameExtents();if(r.multiply(T),s.copy(v.mapSize),(r.x>u||r.y>u)&&(r.x>u&&(s.x=Math.floor(u/T.x),r.x=s.x*T.x,v.mapSize.x=s.x),r.y>u&&(s.y=Math.floor(u/T.y),r.y=s.y*T.y,v.mapSize.y=s.y)),v.map===null||O===!0||$===!0){const N=this.type!==Kt?{minFilter:Pt,magFilter:Pt}:{};v.map!==null&&v.map.dispose(),v.map=new Un(r.x,r.y,N),v.map.texture.name=te.name+".shadowMap",v.camera.updateProjectionMatrix()}i.setRenderTarget(v.map),i.clear();const U=v.getViewportCount();for(let N=0;N<U;N++){const V=v.getViewport(N);o.set(s.x*V.x,s.y*V.y,s.x*V.z,s.y*V.w),H.viewport(o),v.updateMatrices(te,N),n=v.getFrustum(),A(P,Y,v.camera,te,this.type)}v.isPointLightShadow!==!0&&this.type===Kt&&w(v,Y),v.needsUpdate=!1}p=this.type,m.needsUpdate=!1,i.setRenderTarget(E,b,C)};function w(I,P){const Y=e.update(x);d.defines.VSM_SAMPLES!==I.blurSamples&&(d.defines.VSM_SAMPLES=I.blurSamples,f.defines.VSM_SAMPLES=I.blurSamples,d.needsUpdate=!0,f.needsUpdate=!0),I.mapPass===null&&(I.mapPass=new Un(r.x,r.y)),d.uniforms.shadow_pass.value=I.map.texture,d.uniforms.resolution.value=I.mapSize,d.uniforms.radius.value=I.radius,i.setRenderTarget(I.mapPass),i.clear(),i.renderBufferDirect(P,null,Y,d,x,null),f.uniforms.shadow_pass.value=I.mapPass.texture,f.uniforms.resolution.value=I.mapSize,f.uniforms.radius.value=I.radius,i.setRenderTarget(I.map),i.clear(),i.renderBufferDirect(P,null,Y,f,x,null)}function S(I,P,Y,E){let b=null;const C=Y.isPointLight===!0?I.customDistanceMaterial:I.customDepthMaterial;if(C!==void 0)b=C;else if(b=Y.isPointLight===!0?l:a,i.localClippingEnabled&&P.clipShadows===!0&&Array.isArray(P.clippingPlanes)&&P.clippingPlanes.length!==0||P.displacementMap&&P.displacementScale!==0||P.alphaMap&&P.alphaTest>0||P.map&&P.alphaTest>0){const H=b.uuid,O=P.uuid;let $=c[H];$===void 0&&($={},c[H]=$);let z=$[O];z===void 0&&(z=b.clone(),$[O]=z,P.addEventListener("dispose",k)),b=z}if(b.visible=P.visible,b.wireframe=P.wireframe,E===Kt?b.side=P.shadowSide!==null?P.shadowSide:P.side:b.side=P.shadowSide!==null?P.shadowSide:h[P.side],b.alphaMap=P.alphaMap,b.alphaTest=P.alphaTest,b.map=P.map,b.clipShadows=P.clipShadows,b.clippingPlanes=P.clippingPlanes,b.clipIntersection=P.clipIntersection,b.displacementMap=P.displacementMap,b.displacementScale=P.displacementScale,b.displacementBias=P.displacementBias,b.wireframeLinewidth=P.wireframeLinewidth,b.linewidth=P.linewidth,Y.isPointLight===!0&&b.isMeshDistanceMaterial===!0){const H=i.properties.get(b);H.light=Y}return b}function A(I,P,Y,E,b){if(I.visible===!1)return;if(I.layers.test(P.layers)&&(I.isMesh||I.isLine||I.isPoints)&&(I.castShadow||I.receiveShadow&&b===Kt)&&(!I.frustumCulled||n.intersectsObject(I))){I.modelViewMatrix.multiplyMatrices(Y.matrixWorldInverse,I.matrixWorld);const O=e.update(I),$=I.material;if(Array.isArray($)){const z=O.groups;for(let q=0,te=z.length;q<te;q++){const v=z[q],T=$[v.materialIndex];if(T&&T.visible){const U=S(I,T,E,b);I.onBeforeShadow(i,I,P,Y,O,U,v),i.renderBufferDirect(Y,null,O,U,I,v),I.onAfterShadow(i,I,P,Y,O,U,v)}}}else if($.visible){const z=S(I,$,E,b);I.onBeforeShadow(i,I,P,Y,O,z,null),i.renderBufferDirect(Y,null,O,z,I,null),I.onAfterShadow(i,I,P,Y,O,z,null)}}const H=I.children;for(let O=0,$=H.length;O<$;O++)A(H[O],P,Y,E,b)}function k(I){I.target.removeEventListener("dispose",k);for(const Y in c){const E=c[Y],b=I.target.uuid;b in E&&(E[b].dispose(),delete E[b])}}}function B_(i){function e(){let _=!1;const ne=new ht;let Z=null;const ie=new ht(0,0,0,0);return{setMask:function(se){Z!==se&&!_&&(i.colorMask(se,se,se,se),Z=se)},setLocked:function(se){_=se},setClear:function(se,Ee,Ue,et,it){it===!0&&(se*=et,Ee*=et,Ue*=et),ne.set(se,Ee,Ue,et),ie.equals(ne)===!1&&(i.clearColor(se,Ee,Ue,et),ie.copy(ne))},reset:function(){_=!1,Z=null,ie.set(-1,0,0,0)}}}function t(){let _=!1,ne=null,Z=null,ie=null;return{setTest:function(se){se?j(i.DEPTH_TEST):X(i.DEPTH_TEST)},setMask:function(se){ne!==se&&!_&&(i.depthMask(se),ne=se)},setFunc:function(se){if(Z!==se){switch(se){case nd:i.depthFunc(i.NEVER);break;case id:i.depthFunc(i.ALWAYS);break;case rd:i.depthFunc(i.LESS);break;case dr:i.depthFunc(i.LEQUAL);break;case sd:i.depthFunc(i.EQUAL);break;case ad:i.depthFunc(i.GEQUAL);break;case od:i.depthFunc(i.GREATER);break;case ld:i.depthFunc(i.NOTEQUAL);break;default:i.depthFunc(i.LEQUAL)}Z=se}},setLocked:function(se){_=se},setClear:function(se){ie!==se&&(i.clearDepth(se),ie=se)},reset:function(){_=!1,ne=null,Z=null,ie=null}}}function n(){let _=!1,ne=null,Z=null,ie=null,se=null,Ee=null,Ue=null,et=null,it=null;return{setTest:function(We){_||(We?j(i.STENCIL_TEST):X(i.STENCIL_TEST))},setMask:function(We){ne!==We&&!_&&(i.stencilMask(We),ne=We)},setFunc:function(We,rt,st){(Z!==We||ie!==rt||se!==st)&&(i.stencilFunc(We,rt,st),Z=We,ie=rt,se=st)},setOp:function(We,rt,st){(Ee!==We||Ue!==rt||et!==st)&&(i.stencilOp(We,rt,st),Ee=We,Ue=rt,et=st)},setLocked:function(We){_=We},setClear:function(We){it!==We&&(i.clearStencil(We),it=We)},reset:function(){_=!1,ne=null,Z=null,ie=null,se=null,Ee=null,Ue=null,et=null,it=null}}}const r=new e,s=new t,o=new n,a=new WeakMap,l=new WeakMap;let c={},u={},h=new WeakMap,d=[],f=null,g=!1,x=null,m=null,p=null,w=null,S=null,A=null,k=null,I=new He(0,0,0),P=0,Y=!1,E=null,b=null,C=null,H=null,O=null;const $=i.getParameter(i.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let z=!1,q=0;const te=i.getParameter(i.VERSION);te.indexOf("WebGL")!==-1?(q=parseFloat(/^WebGL (\d)/.exec(te)[1]),z=q>=1):te.indexOf("OpenGL ES")!==-1&&(q=parseFloat(/^OpenGL ES (\d)/.exec(te)[1]),z=q>=2);let v=null,T={};const U=i.getParameter(i.SCISSOR_BOX),N=i.getParameter(i.VIEWPORT),V=new ht().fromArray(U),K=new ht().fromArray(N);function D(_,ne,Z,ie){const se=new Uint8Array(4),Ee=i.createTexture();i.bindTexture(_,Ee),i.texParameteri(_,i.TEXTURE_MIN_FILTER,i.NEAREST),i.texParameteri(_,i.TEXTURE_MAG_FILTER,i.NEAREST);for(let Ue=0;Ue<Z;Ue++)_===i.TEXTURE_3D||_===i.TEXTURE_2D_ARRAY?i.texImage3D(ne,0,i.RGBA,1,1,ie,0,i.RGBA,i.UNSIGNED_BYTE,se):i.texImage2D(ne+Ue,0,i.RGBA,1,1,0,i.RGBA,i.UNSIGNED_BYTE,se);return Ee}const B={};B[i.TEXTURE_2D]=D(i.TEXTURE_2D,i.TEXTURE_2D,1),B[i.TEXTURE_CUBE_MAP]=D(i.TEXTURE_CUBE_MAP,i.TEXTURE_CUBE_MAP_POSITIVE_X,6),B[i.TEXTURE_2D_ARRAY]=D(i.TEXTURE_2D_ARRAY,i.TEXTURE_2D_ARRAY,1,1),B[i.TEXTURE_3D]=D(i.TEXTURE_3D,i.TEXTURE_3D,1,1),r.setClear(0,0,0,1),s.setClear(1),o.setClear(0),j(i.DEPTH_TEST),s.setFunc(dr),Me(!1),Be(To),j(i.CULL_FACE),we(dn);function j(_){c[_]!==!0&&(i.enable(_),c[_]=!0)}function X(_){c[_]!==!1&&(i.disable(_),c[_]=!1)}function le(_,ne){return u[_]!==ne?(i.bindFramebuffer(_,ne),u[_]=ne,_===i.DRAW_FRAMEBUFFER&&(u[i.FRAMEBUFFER]=ne),_===i.FRAMEBUFFER&&(u[i.DRAW_FRAMEBUFFER]=ne),!0):!1}function ve(_,ne){let Z=d,ie=!1;if(_){Z=h.get(ne),Z===void 0&&(Z=[],h.set(ne,Z));const se=_.textures;if(Z.length!==se.length||Z[0]!==i.COLOR_ATTACHMENT0){for(let Ee=0,Ue=se.length;Ee<Ue;Ee++)Z[Ee]=i.COLOR_ATTACHMENT0+Ee;Z.length=se.length,ie=!0}}else Z[0]!==i.BACK&&(Z[0]=i.BACK,ie=!0);ie&&i.drawBuffers(Z)}function ge(_){return f!==_?(i.useProgram(_),f=_,!0):!1}const L={[Cn]:i.FUNC_ADD,[kh]:i.FUNC_SUBTRACT,[zh]:i.FUNC_REVERSE_SUBTRACT};L[Vh]=i.MIN,L[Gh]=i.MAX;const xe={[Hh]:i.ZERO,[Wh]:i.ONE,[Xh]:i.SRC_COLOR,[sa]:i.SRC_ALPHA,[Zh]:i.SRC_ALPHA_SATURATE,[$h]:i.DST_COLOR,[Yh]:i.DST_ALPHA,[qh]:i.ONE_MINUS_SRC_COLOR,[aa]:i.ONE_MINUS_SRC_ALPHA,[Kh]:i.ONE_MINUS_DST_COLOR,[jh]:i.ONE_MINUS_DST_ALPHA,[Jh]:i.CONSTANT_COLOR,[Qh]:i.ONE_MINUS_CONSTANT_COLOR,[ed]:i.CONSTANT_ALPHA,[td]:i.ONE_MINUS_CONSTANT_ALPHA};function we(_,ne,Z,ie,se,Ee,Ue,et,it,We){if(_===dn){g===!0&&(X(i.BLEND),g=!1);return}if(g===!1&&(j(i.BLEND),g=!0),_!==Bh){if(_!==x||We!==Y){if((m!==Cn||S!==Cn)&&(i.blendEquation(i.FUNC_ADD),m=Cn,S=Cn),We)switch(_){case si:i.blendFuncSeparate(i.ONE,i.ONE_MINUS_SRC_ALPHA,i.ONE,i.ONE_MINUS_SRC_ALPHA);break;case Ao:i.blendFunc(i.ONE,i.ONE);break;case Co:i.blendFuncSeparate(i.ZERO,i.ONE_MINUS_SRC_COLOR,i.ZERO,i.ONE);break;case Ro:i.blendFuncSeparate(i.ZERO,i.SRC_COLOR,i.ZERO,i.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",_);break}else switch(_){case si:i.blendFuncSeparate(i.SRC_ALPHA,i.ONE_MINUS_SRC_ALPHA,i.ONE,i.ONE_MINUS_SRC_ALPHA);break;case Ao:i.blendFunc(i.SRC_ALPHA,i.ONE);break;case Co:i.blendFuncSeparate(i.ZERO,i.ONE_MINUS_SRC_COLOR,i.ZERO,i.ONE);break;case Ro:i.blendFunc(i.ZERO,i.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",_);break}p=null,w=null,A=null,k=null,I.set(0,0,0),P=0,x=_,Y=We}return}se=se||ne,Ee=Ee||Z,Ue=Ue||ie,(ne!==m||se!==S)&&(i.blendEquationSeparate(L[ne],L[se]),m=ne,S=se),(Z!==p||ie!==w||Ee!==A||Ue!==k)&&(i.blendFuncSeparate(xe[Z],xe[ie],xe[Ee],xe[Ue]),p=Z,w=ie,A=Ee,k=Ue),(et.equals(I)===!1||it!==P)&&(i.blendColor(et.r,et.g,et.b,it),I.copy(et),P=it),x=_,Y=!1}function Xe(_,ne){_.side===Zt?X(i.CULL_FACE):j(i.CULL_FACE);let Z=_.side===yt;ne&&(Z=!Z),Me(Z),_.blending===si&&_.transparent===!1?we(dn):we(_.blending,_.blendEquation,_.blendSrc,_.blendDst,_.blendEquationAlpha,_.blendSrcAlpha,_.blendDstAlpha,_.blendColor,_.blendAlpha,_.premultipliedAlpha),s.setFunc(_.depthFunc),s.setTest(_.depthTest),s.setMask(_.depthWrite),r.setMask(_.colorWrite);const ie=_.stencilWrite;o.setTest(ie),ie&&(o.setMask(_.stencilWriteMask),o.setFunc(_.stencilFunc,_.stencilRef,_.stencilFuncMask),o.setOp(_.stencilFail,_.stencilZFail,_.stencilZPass)),Ae(_.polygonOffset,_.polygonOffsetFactor,_.polygonOffsetUnits),_.alphaToCoverage===!0?j(i.SAMPLE_ALPHA_TO_COVERAGE):X(i.SAMPLE_ALPHA_TO_COVERAGE)}function Me(_){E!==_&&(_?i.frontFace(i.CW):i.frontFace(i.CCW),E=_)}function Be(_){_!==Nh?(j(i.CULL_FACE),_!==b&&(_===To?i.cullFace(i.BACK):_===Oh?i.cullFace(i.FRONT):i.cullFace(i.FRONT_AND_BACK))):X(i.CULL_FACE),b=_}function Ie(_){_!==C&&(z&&i.lineWidth(_),C=_)}function Ae(_,ne,Z){_?(j(i.POLYGON_OFFSET_FILL),(H!==ne||O!==Z)&&(i.polygonOffset(ne,Z),H=ne,O=Z)):X(i.POLYGON_OFFSET_FILL)}function Ve(_){_?j(i.SCISSOR_TEST):X(i.SCISSOR_TEST)}function R(_){_===void 0&&(_=i.TEXTURE0+$-1),v!==_&&(i.activeTexture(_),v=_)}function y(_,ne,Z){Z===void 0&&(v===null?Z=i.TEXTURE0+$-1:Z=v);let ie=T[Z];ie===void 0&&(ie={type:void 0,texture:void 0},T[Z]=ie),(ie.type!==_||ie.texture!==ne)&&(v!==Z&&(i.activeTexture(Z),v=Z),i.bindTexture(_,ne||B[_]),ie.type=_,ie.texture=ne)}function ee(){const _=T[v];_!==void 0&&_.type!==void 0&&(i.bindTexture(_.type,null),_.type=void 0,_.texture=void 0)}function re(){try{i.compressedTexImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function ae(){try{i.compressedTexImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function oe(){try{i.texSubImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function be(){try{i.texSubImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function de(){try{i.compressedTexSubImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function he(){try{i.compressedTexSubImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function Ne(){try{i.texStorage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function ce(){try{i.texStorage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function ye(){try{i.texImage2D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function Ge(){try{i.texImage3D.apply(i,arguments)}catch(_){console.error("THREE.WebGLState:",_)}}function Re(_){V.equals(_)===!1&&(i.scissor(_.x,_.y,_.z,_.w),V.copy(_))}function pe(_){K.equals(_)===!1&&(i.viewport(_.x,_.y,_.z,_.w),K.copy(_))}function Oe(_,ne){let Z=l.get(ne);Z===void 0&&(Z=new WeakMap,l.set(ne,Z));let ie=Z.get(_);ie===void 0&&(ie=i.getUniformBlockIndex(ne,_.name),Z.set(_,ie))}function Fe(_,ne){const ie=l.get(ne).get(_);a.get(ne)!==ie&&(i.uniformBlockBinding(ne,ie,_.__bindingPointIndex),a.set(ne,ie))}function Qe(){i.disable(i.BLEND),i.disable(i.CULL_FACE),i.disable(i.DEPTH_TEST),i.disable(i.POLYGON_OFFSET_FILL),i.disable(i.SCISSOR_TEST),i.disable(i.STENCIL_TEST),i.disable(i.SAMPLE_ALPHA_TO_COVERAGE),i.blendEquation(i.FUNC_ADD),i.blendFunc(i.ONE,i.ZERO),i.blendFuncSeparate(i.ONE,i.ZERO,i.ONE,i.ZERO),i.blendColor(0,0,0,0),i.colorMask(!0,!0,!0,!0),i.clearColor(0,0,0,0),i.depthMask(!0),i.depthFunc(i.LESS),i.clearDepth(1),i.stencilMask(4294967295),i.stencilFunc(i.ALWAYS,0,4294967295),i.stencilOp(i.KEEP,i.KEEP,i.KEEP),i.clearStencil(0),i.cullFace(i.BACK),i.frontFace(i.CCW),i.polygonOffset(0,0),i.activeTexture(i.TEXTURE0),i.bindFramebuffer(i.FRAMEBUFFER,null),i.bindFramebuffer(i.DRAW_FRAMEBUFFER,null),i.bindFramebuffer(i.READ_FRAMEBUFFER,null),i.useProgram(null),i.lineWidth(1),i.scissor(0,0,i.canvas.width,i.canvas.height),i.viewport(0,0,i.canvas.width,i.canvas.height),c={},v=null,T={},u={},h=new WeakMap,d=[],f=null,g=!1,x=null,m=null,p=null,w=null,S=null,A=null,k=null,I=new He(0,0,0),P=0,Y=!1,E=null,b=null,C=null,H=null,O=null,V.set(0,0,i.canvas.width,i.canvas.height),K.set(0,0,i.canvas.width,i.canvas.height),r.reset(),s.reset(),o.reset()}return{buffers:{color:r,depth:s,stencil:o},enable:j,disable:X,bindFramebuffer:le,drawBuffers:ve,useProgram:ge,setBlending:we,setMaterial:Xe,setFlipSided:Me,setCullFace:Be,setLineWidth:Ie,setPolygonOffset:Ae,setScissorTest:Ve,activeTexture:R,bindTexture:y,unbindTexture:ee,compressedTexImage2D:re,compressedTexImage3D:ae,texImage2D:ye,texImage3D:Ge,updateUBOMapping:Oe,uniformBlockBinding:Fe,texStorage2D:Ne,texStorage3D:ce,texSubImage2D:oe,texSubImage3D:be,compressedTexSubImage2D:de,compressedTexSubImage3D:he,scissor:Re,viewport:pe,reset:Qe}}function k_(i,e,t,n,r,s,o){const a=e.has("WEBGL_multisampled_render_to_texture")?e.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),c=new De,u=new WeakMap;let h;const d=new WeakMap;let f=!1;try{f=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(R,y){return f?new OffscreenCanvas(R,y):vr("canvas")}function x(R,y,ee){let re=1;const ae=Ve(R);if((ae.width>ee||ae.height>ee)&&(re=ee/Math.max(ae.width,ae.height)),re<1)if(typeof HTMLImageElement<"u"&&R instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&R instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&R instanceof ImageBitmap||typeof VideoFrame<"u"&&R instanceof VideoFrame){const oe=Math.floor(re*ae.width),be=Math.floor(re*ae.height);h===void 0&&(h=g(oe,be));const de=y?g(oe,be):h;return de.width=oe,de.height=be,de.getContext("2d").drawImage(R,0,0,oe,be),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+ae.width+"x"+ae.height+") to ("+oe+"x"+be+")."),de}else return"data"in R&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+ae.width+"x"+ae.height+")."),R;return R}function m(R){return R.generateMipmaps&&R.minFilter!==Pt&&R.minFilter!==Nt}function p(R){i.generateMipmap(R)}function w(R,y,ee,re,ae=!1){if(R!==null){if(i[R]!==void 0)return i[R];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+R+"'")}let oe=y;if(y===i.RED&&(ee===i.FLOAT&&(oe=i.R32F),ee===i.HALF_FLOAT&&(oe=i.R16F),ee===i.UNSIGNED_BYTE&&(oe=i.R8)),y===i.RED_INTEGER&&(ee===i.UNSIGNED_BYTE&&(oe=i.R8UI),ee===i.UNSIGNED_SHORT&&(oe=i.R16UI),ee===i.UNSIGNED_INT&&(oe=i.R32UI),ee===i.BYTE&&(oe=i.R8I),ee===i.SHORT&&(oe=i.R16I),ee===i.INT&&(oe=i.R32I)),y===i.RG&&(ee===i.FLOAT&&(oe=i.RG32F),ee===i.HALF_FLOAT&&(oe=i.RG16F),ee===i.UNSIGNED_BYTE&&(oe=i.RG8)),y===i.RG_INTEGER&&(ee===i.UNSIGNED_BYTE&&(oe=i.RG8UI),ee===i.UNSIGNED_SHORT&&(oe=i.RG16UI),ee===i.UNSIGNED_INT&&(oe=i.RG32UI),ee===i.BYTE&&(oe=i.RG8I),ee===i.SHORT&&(oe=i.RG16I),ee===i.INT&&(oe=i.RG32I)),y===i.RGB&&ee===i.UNSIGNED_INT_5_9_9_9_REV&&(oe=i.RGB9_E5),y===i.RGBA){const be=ae?pr:je.getTransfer(re);ee===i.FLOAT&&(oe=i.RGBA32F),ee===i.HALF_FLOAT&&(oe=i.RGBA16F),ee===i.UNSIGNED_BYTE&&(oe=be===Ke?i.SRGB8_ALPHA8:i.RGBA8),ee===i.UNSIGNED_SHORT_4_4_4_4&&(oe=i.RGBA4),ee===i.UNSIGNED_SHORT_5_5_5_1&&(oe=i.RGB5_A1)}return(oe===i.R16F||oe===i.R32F||oe===i.RG16F||oe===i.RG32F||oe===i.RGBA16F||oe===i.RGBA32F)&&e.get("EXT_color_buffer_float"),oe}function S(R,y){let ee;return R?y===null||y===ui||y===hi?ee=i.DEPTH24_STENCIL8:y===un?ee=i.DEPTH32F_STENCIL8:y===fr&&(ee=i.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):y===null||y===ui||y===hi?ee=i.DEPTH_COMPONENT24:y===un?ee=i.DEPTH_COMPONENT32F:y===fr&&(ee=i.DEPTH_COMPONENT16),ee}function A(R,y){return m(R)===!0||R.isFramebufferTexture&&R.minFilter!==Pt&&R.minFilter!==Nt?Math.log2(Math.max(y.width,y.height))+1:R.mipmaps!==void 0&&R.mipmaps.length>0?R.mipmaps.length:R.isCompressedTexture&&Array.isArray(R.image)?y.mipmaps.length:1}function k(R){const y=R.target;y.removeEventListener("dispose",k),P(y),y.isVideoTexture&&u.delete(y)}function I(R){const y=R.target;y.removeEventListener("dispose",I),E(y)}function P(R){const y=n.get(R);if(y.__webglInit===void 0)return;const ee=R.source,re=d.get(ee);if(re){const ae=re[y.__cacheKey];ae.usedTimes--,ae.usedTimes===0&&Y(R),Object.keys(re).length===0&&d.delete(ee)}n.remove(R)}function Y(R){const y=n.get(R);i.deleteTexture(y.__webglTexture);const ee=R.source,re=d.get(ee);delete re[y.__cacheKey],o.memory.textures--}function E(R){const y=n.get(R);if(R.depthTexture&&R.depthTexture.dispose(),R.isWebGLCubeRenderTarget)for(let re=0;re<6;re++){if(Array.isArray(y.__webglFramebuffer[re]))for(let ae=0;ae<y.__webglFramebuffer[re].length;ae++)i.deleteFramebuffer(y.__webglFramebuffer[re][ae]);else i.deleteFramebuffer(y.__webglFramebuffer[re]);y.__webglDepthbuffer&&i.deleteRenderbuffer(y.__webglDepthbuffer[re])}else{if(Array.isArray(y.__webglFramebuffer))for(let re=0;re<y.__webglFramebuffer.length;re++)i.deleteFramebuffer(y.__webglFramebuffer[re]);else i.deleteFramebuffer(y.__webglFramebuffer);if(y.__webglDepthbuffer&&i.deleteRenderbuffer(y.__webglDepthbuffer),y.__webglMultisampledFramebuffer&&i.deleteFramebuffer(y.__webglMultisampledFramebuffer),y.__webglColorRenderbuffer)for(let re=0;re<y.__webglColorRenderbuffer.length;re++)y.__webglColorRenderbuffer[re]&&i.deleteRenderbuffer(y.__webglColorRenderbuffer[re]);y.__webglDepthRenderbuffer&&i.deleteRenderbuffer(y.__webglDepthRenderbuffer)}const ee=R.textures;for(let re=0,ae=ee.length;re<ae;re++){const oe=n.get(ee[re]);oe.__webglTexture&&(i.deleteTexture(oe.__webglTexture),o.memory.textures--),n.remove(ee[re])}n.remove(R)}let b=0;function C(){b=0}function H(){const R=b;return R>=r.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+R+" texture units while this GPU supports only "+r.maxTextures),b+=1,R}function O(R){const y=[];return y.push(R.wrapS),y.push(R.wrapT),y.push(R.wrapR||0),y.push(R.magFilter),y.push(R.minFilter),y.push(R.anisotropy),y.push(R.internalFormat),y.push(R.format),y.push(R.type),y.push(R.generateMipmaps),y.push(R.premultiplyAlpha),y.push(R.flipY),y.push(R.unpackAlignment),y.push(R.colorSpace),y.join()}function $(R,y){const ee=n.get(R);if(R.isVideoTexture&&Ie(R),R.isRenderTargetTexture===!1&&R.version>0&&ee.__version!==R.version){const re=R.image;if(re===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(re.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{K(ee,R,y);return}}t.bindTexture(i.TEXTURE_2D,ee.__webglTexture,i.TEXTURE0+y)}function z(R,y){const ee=n.get(R);if(R.version>0&&ee.__version!==R.version){K(ee,R,y);return}t.bindTexture(i.TEXTURE_2D_ARRAY,ee.__webglTexture,i.TEXTURE0+y)}function q(R,y){const ee=n.get(R);if(R.version>0&&ee.__version!==R.version){K(ee,R,y);return}t.bindTexture(i.TEXTURE_3D,ee.__webglTexture,i.TEXTURE0+y)}function te(R,y){const ee=n.get(R);if(R.version>0&&ee.__version!==R.version){D(ee,R,y);return}t.bindTexture(i.TEXTURE_CUBE_MAP,ee.__webglTexture,i.TEXTURE0+y)}const v={[ca]:i.REPEAT,[Pn]:i.CLAMP_TO_EDGE,[ua]:i.MIRRORED_REPEAT},T={[Pt]:i.NEAREST,[vd]:i.NEAREST_MIPMAP_NEAREST,[Ni]:i.NEAREST_MIPMAP_LINEAR,[Nt]:i.LINEAR,[bs]:i.LINEAR_MIPMAP_NEAREST,[Ln]:i.LINEAR_MIPMAP_LINEAR},U={[Ud]:i.NEVER,[Bd]:i.ALWAYS,[Dd]:i.LESS,[Fc]:i.LEQUAL,[Id]:i.EQUAL,[Fd]:i.GEQUAL,[Nd]:i.GREATER,[Od]:i.NOTEQUAL};function N(R,y){if(y.type===un&&e.has("OES_texture_float_linear")===!1&&(y.magFilter===Nt||y.magFilter===bs||y.magFilter===Ni||y.magFilter===Ln||y.minFilter===Nt||y.minFilter===bs||y.minFilter===Ni||y.minFilter===Ln)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),i.texParameteri(R,i.TEXTURE_WRAP_S,v[y.wrapS]),i.texParameteri(R,i.TEXTURE_WRAP_T,v[y.wrapT]),(R===i.TEXTURE_3D||R===i.TEXTURE_2D_ARRAY)&&i.texParameteri(R,i.TEXTURE_WRAP_R,v[y.wrapR]),i.texParameteri(R,i.TEXTURE_MAG_FILTER,T[y.magFilter]),i.texParameteri(R,i.TEXTURE_MIN_FILTER,T[y.minFilter]),y.compareFunction&&(i.texParameteri(R,i.TEXTURE_COMPARE_MODE,i.COMPARE_REF_TO_TEXTURE),i.texParameteri(R,i.TEXTURE_COMPARE_FUNC,U[y.compareFunction])),e.has("EXT_texture_filter_anisotropic")===!0){if(y.magFilter===Pt||y.minFilter!==Ni&&y.minFilter!==Ln||y.type===un&&e.has("OES_texture_float_linear")===!1)return;if(y.anisotropy>1||n.get(y).__currentAnisotropy){const ee=e.get("EXT_texture_filter_anisotropic");i.texParameterf(R,ee.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(y.anisotropy,r.getMaxAnisotropy())),n.get(y).__currentAnisotropy=y.anisotropy}}}function V(R,y){let ee=!1;R.__webglInit===void 0&&(R.__webglInit=!0,y.addEventListener("dispose",k));const re=y.source;let ae=d.get(re);ae===void 0&&(ae={},d.set(re,ae));const oe=O(y);if(oe!==R.__cacheKey){ae[oe]===void 0&&(ae[oe]={texture:i.createTexture(),usedTimes:0},o.memory.textures++,ee=!0),ae[oe].usedTimes++;const be=ae[R.__cacheKey];be!==void 0&&(ae[R.__cacheKey].usedTimes--,be.usedTimes===0&&Y(y)),R.__cacheKey=oe,R.__webglTexture=ae[oe].texture}return ee}function K(R,y,ee){let re=i.TEXTURE_2D;(y.isDataArrayTexture||y.isCompressedArrayTexture)&&(re=i.TEXTURE_2D_ARRAY),y.isData3DTexture&&(re=i.TEXTURE_3D);const ae=V(R,y),oe=y.source;t.bindTexture(re,R.__webglTexture,i.TEXTURE0+ee);const be=n.get(oe);if(oe.version!==be.__version||ae===!0){t.activeTexture(i.TEXTURE0+ee);const de=je.getPrimaries(je.workingColorSpace),he=y.colorSpace===cn?null:je.getPrimaries(y.colorSpace),Ne=y.colorSpace===cn||de===he?i.NONE:i.BROWSER_DEFAULT_WEBGL;i.pixelStorei(i.UNPACK_FLIP_Y_WEBGL,y.flipY),i.pixelStorei(i.UNPACK_PREMULTIPLY_ALPHA_WEBGL,y.premultiplyAlpha),i.pixelStorei(i.UNPACK_ALIGNMENT,y.unpackAlignment),i.pixelStorei(i.UNPACK_COLORSPACE_CONVERSION_WEBGL,Ne);let ce=x(y.image,!1,r.maxTextureSize);ce=Ae(y,ce);const ye=s.convert(y.format,y.colorSpace),Ge=s.convert(y.type);let Re=w(y.internalFormat,ye,Ge,y.colorSpace,y.isVideoTexture);N(re,y);let pe;const Oe=y.mipmaps,Fe=y.isVideoTexture!==!0,Qe=be.__version===void 0||ae===!0,_=oe.dataReady,ne=A(y,ce);if(y.isDepthTexture)Re=S(y.format===di,y.type),Qe&&(Fe?t.texStorage2D(i.TEXTURE_2D,1,Re,ce.width,ce.height):t.texImage2D(i.TEXTURE_2D,0,Re,ce.width,ce.height,0,ye,Ge,null));else if(y.isDataTexture)if(Oe.length>0){Fe&&Qe&&t.texStorage2D(i.TEXTURE_2D,ne,Re,Oe[0].width,Oe[0].height);for(let Z=0,ie=Oe.length;Z<ie;Z++)pe=Oe[Z],Fe?_&&t.texSubImage2D(i.TEXTURE_2D,Z,0,0,pe.width,pe.height,ye,Ge,pe.data):t.texImage2D(i.TEXTURE_2D,Z,Re,pe.width,pe.height,0,ye,Ge,pe.data);y.generateMipmaps=!1}else Fe?(Qe&&t.texStorage2D(i.TEXTURE_2D,ne,Re,ce.width,ce.height),_&&t.texSubImage2D(i.TEXTURE_2D,0,0,0,ce.width,ce.height,ye,Ge,ce.data)):t.texImage2D(i.TEXTURE_2D,0,Re,ce.width,ce.height,0,ye,Ge,ce.data);else if(y.isCompressedTexture)if(y.isCompressedArrayTexture){Fe&&Qe&&t.texStorage3D(i.TEXTURE_2D_ARRAY,ne,Re,Oe[0].width,Oe[0].height,ce.depth);for(let Z=0,ie=Oe.length;Z<ie;Z++)if(pe=Oe[Z],y.format!==zt)if(ye!==null)if(Fe){if(_)if(y.layerUpdates.size>0){for(const se of y.layerUpdates){const Ee=pe.width*pe.height;t.compressedTexSubImage3D(i.TEXTURE_2D_ARRAY,Z,0,0,se,pe.width,pe.height,1,ye,pe.data.slice(Ee*se,Ee*(se+1)),0,0)}y.clearLayerUpdates()}else t.compressedTexSubImage3D(i.TEXTURE_2D_ARRAY,Z,0,0,0,pe.width,pe.height,ce.depth,ye,pe.data,0,0)}else t.compressedTexImage3D(i.TEXTURE_2D_ARRAY,Z,Re,pe.width,pe.height,ce.depth,0,pe.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else Fe?_&&t.texSubImage3D(i.TEXTURE_2D_ARRAY,Z,0,0,0,pe.width,pe.height,ce.depth,ye,Ge,pe.data):t.texImage3D(i.TEXTURE_2D_ARRAY,Z,Re,pe.width,pe.height,ce.depth,0,ye,Ge,pe.data)}else{Fe&&Qe&&t.texStorage2D(i.TEXTURE_2D,ne,Re,Oe[0].width,Oe[0].height);for(let Z=0,ie=Oe.length;Z<ie;Z++)pe=Oe[Z],y.format!==zt?ye!==null?Fe?_&&t.compressedTexSubImage2D(i.TEXTURE_2D,Z,0,0,pe.width,pe.height,ye,pe.data):t.compressedTexImage2D(i.TEXTURE_2D,Z,Re,pe.width,pe.height,0,pe.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):Fe?_&&t.texSubImage2D(i.TEXTURE_2D,Z,0,0,pe.width,pe.height,ye,Ge,pe.data):t.texImage2D(i.TEXTURE_2D,Z,Re,pe.width,pe.height,0,ye,Ge,pe.data)}else if(y.isDataArrayTexture)if(Fe){if(Qe&&t.texStorage3D(i.TEXTURE_2D_ARRAY,ne,Re,ce.width,ce.height,ce.depth),_)if(y.layerUpdates.size>0){let Z;switch(Ge){case i.UNSIGNED_BYTE:switch(ye){case i.ALPHA:Z=1;break;case i.LUMINANCE:Z=1;break;case i.LUMINANCE_ALPHA:Z=2;break;case i.RGB:Z=3;break;case i.RGBA:Z=4;break;default:throw new Error(`Unknown texel size for format ${ye}.`)}break;case i.UNSIGNED_SHORT_4_4_4_4:case i.UNSIGNED_SHORT_5_5_5_1:case i.UNSIGNED_SHORT_5_6_5:Z=1;break;default:throw new Error(`Unknown texel size for type ${Ge}.`)}const ie=ce.width*ce.height*Z;for(const se of y.layerUpdates)t.texSubImage3D(i.TEXTURE_2D_ARRAY,0,0,0,se,ce.width,ce.height,1,ye,Ge,ce.data.slice(ie*se,ie*(se+1)));y.clearLayerUpdates()}else t.texSubImage3D(i.TEXTURE_2D_ARRAY,0,0,0,0,ce.width,ce.height,ce.depth,ye,Ge,ce.data)}else t.texImage3D(i.TEXTURE_2D_ARRAY,0,Re,ce.width,ce.height,ce.depth,0,ye,Ge,ce.data);else if(y.isData3DTexture)Fe?(Qe&&t.texStorage3D(i.TEXTURE_3D,ne,Re,ce.width,ce.height,ce.depth),_&&t.texSubImage3D(i.TEXTURE_3D,0,0,0,0,ce.width,ce.height,ce.depth,ye,Ge,ce.data)):t.texImage3D(i.TEXTURE_3D,0,Re,ce.width,ce.height,ce.depth,0,ye,Ge,ce.data);else if(y.isFramebufferTexture){if(Qe)if(Fe)t.texStorage2D(i.TEXTURE_2D,ne,Re,ce.width,ce.height);else{let Z=ce.width,ie=ce.height;for(let se=0;se<ne;se++)t.texImage2D(i.TEXTURE_2D,se,Re,Z,ie,0,ye,Ge,null),Z>>=1,ie>>=1}}else if(Oe.length>0){if(Fe&&Qe){const Z=Ve(Oe[0]);t.texStorage2D(i.TEXTURE_2D,ne,Re,Z.width,Z.height)}for(let Z=0,ie=Oe.length;Z<ie;Z++)pe=Oe[Z],Fe?_&&t.texSubImage2D(i.TEXTURE_2D,Z,0,0,ye,Ge,pe):t.texImage2D(i.TEXTURE_2D,Z,Re,ye,Ge,pe);y.generateMipmaps=!1}else if(Fe){if(Qe){const Z=Ve(ce);t.texStorage2D(i.TEXTURE_2D,ne,Re,Z.width,Z.height)}_&&t.texSubImage2D(i.TEXTURE_2D,0,0,0,ye,Ge,ce)}else t.texImage2D(i.TEXTURE_2D,0,Re,ye,Ge,ce);m(y)&&p(re),be.__version=oe.version,y.onUpdate&&y.onUpdate(y)}R.__version=y.version}function D(R,y,ee){if(y.image.length!==6)return;const re=V(R,y),ae=y.source;t.bindTexture(i.TEXTURE_CUBE_MAP,R.__webglTexture,i.TEXTURE0+ee);const oe=n.get(ae);if(ae.version!==oe.__version||re===!0){t.activeTexture(i.TEXTURE0+ee);const be=je.getPrimaries(je.workingColorSpace),de=y.colorSpace===cn?null:je.getPrimaries(y.colorSpace),he=y.colorSpace===cn||be===de?i.NONE:i.BROWSER_DEFAULT_WEBGL;i.pixelStorei(i.UNPACK_FLIP_Y_WEBGL,y.flipY),i.pixelStorei(i.UNPACK_PREMULTIPLY_ALPHA_WEBGL,y.premultiplyAlpha),i.pixelStorei(i.UNPACK_ALIGNMENT,y.unpackAlignment),i.pixelStorei(i.UNPACK_COLORSPACE_CONVERSION_WEBGL,he);const Ne=y.isCompressedTexture||y.image[0].isCompressedTexture,ce=y.image[0]&&y.image[0].isDataTexture,ye=[];for(let ie=0;ie<6;ie++)!Ne&&!ce?ye[ie]=x(y.image[ie],!0,r.maxCubemapSize):ye[ie]=ce?y.image[ie].image:y.image[ie],ye[ie]=Ae(y,ye[ie]);const Ge=ye[0],Re=s.convert(y.format,y.colorSpace),pe=s.convert(y.type),Oe=w(y.internalFormat,Re,pe,y.colorSpace),Fe=y.isVideoTexture!==!0,Qe=oe.__version===void 0||re===!0,_=ae.dataReady;let ne=A(y,Ge);N(i.TEXTURE_CUBE_MAP,y);let Z;if(Ne){Fe&&Qe&&t.texStorage2D(i.TEXTURE_CUBE_MAP,ne,Oe,Ge.width,Ge.height);for(let ie=0;ie<6;ie++){Z=ye[ie].mipmaps;for(let se=0;se<Z.length;se++){const Ee=Z[se];y.format!==zt?Re!==null?Fe?_&&t.compressedTexSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,0,0,Ee.width,Ee.height,Re,Ee.data):t.compressedTexImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,Oe,Ee.width,Ee.height,0,Ee.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,0,0,Ee.width,Ee.height,Re,pe,Ee.data):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se,Oe,Ee.width,Ee.height,0,Re,pe,Ee.data)}}}else{if(Z=y.mipmaps,Fe&&Qe){Z.length>0&&ne++;const ie=Ve(ye[0]);t.texStorage2D(i.TEXTURE_CUBE_MAP,ne,Oe,ie.width,ie.height)}for(let ie=0;ie<6;ie++)if(ce){Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,0,0,ye[ie].width,ye[ie].height,Re,pe,ye[ie].data):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,Oe,ye[ie].width,ye[ie].height,0,Re,pe,ye[ie].data);for(let se=0;se<Z.length;se++){const Ue=Z[se].image[ie].image;Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,0,0,Ue.width,Ue.height,Re,pe,Ue.data):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,Oe,Ue.width,Ue.height,0,Re,pe,Ue.data)}}else{Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,0,0,Re,pe,ye[ie]):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,0,Oe,Re,pe,ye[ie]);for(let se=0;se<Z.length;se++){const Ee=Z[se];Fe?_&&t.texSubImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,0,0,Re,pe,Ee.image[ie]):t.texImage2D(i.TEXTURE_CUBE_MAP_POSITIVE_X+ie,se+1,Oe,Re,pe,Ee.image[ie])}}}m(y)&&p(i.TEXTURE_CUBE_MAP),oe.__version=ae.version,y.onUpdate&&y.onUpdate(y)}R.__version=y.version}function B(R,y,ee,re,ae,oe){const be=s.convert(ee.format,ee.colorSpace),de=s.convert(ee.type),he=w(ee.internalFormat,be,de,ee.colorSpace);if(!n.get(y).__hasExternalTextures){const ce=Math.max(1,y.width>>oe),ye=Math.max(1,y.height>>oe);ae===i.TEXTURE_3D||ae===i.TEXTURE_2D_ARRAY?t.texImage3D(ae,oe,he,ce,ye,y.depth,0,be,de,null):t.texImage2D(ae,oe,he,ce,ye,0,be,de,null)}t.bindFramebuffer(i.FRAMEBUFFER,R),Be(y)?a.framebufferTexture2DMultisampleEXT(i.FRAMEBUFFER,re,ae,n.get(ee).__webglTexture,0,Me(y)):(ae===i.TEXTURE_2D||ae>=i.TEXTURE_CUBE_MAP_POSITIVE_X&&ae<=i.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&i.framebufferTexture2D(i.FRAMEBUFFER,re,ae,n.get(ee).__webglTexture,oe),t.bindFramebuffer(i.FRAMEBUFFER,null)}function j(R,y,ee){if(i.bindRenderbuffer(i.RENDERBUFFER,R),y.depthBuffer){const re=y.depthTexture,ae=re&&re.isDepthTexture?re.type:null,oe=S(y.stencilBuffer,ae),be=y.stencilBuffer?i.DEPTH_STENCIL_ATTACHMENT:i.DEPTH_ATTACHMENT,de=Me(y);Be(y)?a.renderbufferStorageMultisampleEXT(i.RENDERBUFFER,de,oe,y.width,y.height):ee?i.renderbufferStorageMultisample(i.RENDERBUFFER,de,oe,y.width,y.height):i.renderbufferStorage(i.RENDERBUFFER,oe,y.width,y.height),i.framebufferRenderbuffer(i.FRAMEBUFFER,be,i.RENDERBUFFER,R)}else{const re=y.textures;for(let ae=0;ae<re.length;ae++){const oe=re[ae],be=s.convert(oe.format,oe.colorSpace),de=s.convert(oe.type),he=w(oe.internalFormat,be,de,oe.colorSpace),Ne=Me(y);ee&&Be(y)===!1?i.renderbufferStorageMultisample(i.RENDERBUFFER,Ne,he,y.width,y.height):Be(y)?a.renderbufferStorageMultisampleEXT(i.RENDERBUFFER,Ne,he,y.width,y.height):i.renderbufferStorage(i.RENDERBUFFER,he,y.width,y.height)}}i.bindRenderbuffer(i.RENDERBUFFER,null)}function X(R,y){if(y&&y.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(t.bindFramebuffer(i.FRAMEBUFFER,R),!(y.depthTexture&&y.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");(!n.get(y.depthTexture).__webglTexture||y.depthTexture.image.width!==y.width||y.depthTexture.image.height!==y.height)&&(y.depthTexture.image.width=y.width,y.depthTexture.image.height=y.height,y.depthTexture.needsUpdate=!0),$(y.depthTexture,0);const re=n.get(y.depthTexture).__webglTexture,ae=Me(y);if(y.depthTexture.format===ai)Be(y)?a.framebufferTexture2DMultisampleEXT(i.FRAMEBUFFER,i.DEPTH_ATTACHMENT,i.TEXTURE_2D,re,0,ae):i.framebufferTexture2D(i.FRAMEBUFFER,i.DEPTH_ATTACHMENT,i.TEXTURE_2D,re,0);else if(y.depthTexture.format===di)Be(y)?a.framebufferTexture2DMultisampleEXT(i.FRAMEBUFFER,i.DEPTH_STENCIL_ATTACHMENT,i.TEXTURE_2D,re,0,ae):i.framebufferTexture2D(i.FRAMEBUFFER,i.DEPTH_STENCIL_ATTACHMENT,i.TEXTURE_2D,re,0);else throw new Error("Unknown depthTexture format")}function le(R){const y=n.get(R),ee=R.isWebGLCubeRenderTarget===!0;if(R.depthTexture&&!y.__autoAllocateDepthBuffer){if(ee)throw new Error("target.depthTexture not supported in Cube render targets");X(y.__webglFramebuffer,R)}else if(ee){y.__webglDepthbuffer=[];for(let re=0;re<6;re++)t.bindFramebuffer(i.FRAMEBUFFER,y.__webglFramebuffer[re]),y.__webglDepthbuffer[re]=i.createRenderbuffer(),j(y.__webglDepthbuffer[re],R,!1)}else t.bindFramebuffer(i.FRAMEBUFFER,y.__webglFramebuffer),y.__webglDepthbuffer=i.createRenderbuffer(),j(y.__webglDepthbuffer,R,!1);t.bindFramebuffer(i.FRAMEBUFFER,null)}function ve(R,y,ee){const re=n.get(R);y!==void 0&&B(re.__webglFramebuffer,R,R.texture,i.COLOR_ATTACHMENT0,i.TEXTURE_2D,0),ee!==void 0&&le(R)}function ge(R){const y=R.texture,ee=n.get(R),re=n.get(y);R.addEventListener("dispose",I);const ae=R.textures,oe=R.isWebGLCubeRenderTarget===!0,be=ae.length>1;if(be||(re.__webglTexture===void 0&&(re.__webglTexture=i.createTexture()),re.__version=y.version,o.memory.textures++),oe){ee.__webglFramebuffer=[];for(let de=0;de<6;de++)if(y.mipmaps&&y.mipmaps.length>0){ee.__webglFramebuffer[de]=[];for(let he=0;he<y.mipmaps.length;he++)ee.__webglFramebuffer[de][he]=i.createFramebuffer()}else ee.__webglFramebuffer[de]=i.createFramebuffer()}else{if(y.mipmaps&&y.mipmaps.length>0){ee.__webglFramebuffer=[];for(let de=0;de<y.mipmaps.length;de++)ee.__webglFramebuffer[de]=i.createFramebuffer()}else ee.__webglFramebuffer=i.createFramebuffer();if(be)for(let de=0,he=ae.length;de<he;de++){const Ne=n.get(ae[de]);Ne.__webglTexture===void 0&&(Ne.__webglTexture=i.createTexture(),o.memory.textures++)}if(R.samples>0&&Be(R)===!1){ee.__webglMultisampledFramebuffer=i.createFramebuffer(),ee.__webglColorRenderbuffer=[],t.bindFramebuffer(i.FRAMEBUFFER,ee.__webglMultisampledFramebuffer);for(let de=0;de<ae.length;de++){const he=ae[de];ee.__webglColorRenderbuffer[de]=i.createRenderbuffer(),i.bindRenderbuffer(i.RENDERBUFFER,ee.__webglColorRenderbuffer[de]);const Ne=s.convert(he.format,he.colorSpace),ce=s.convert(he.type),ye=w(he.internalFormat,Ne,ce,he.colorSpace,R.isXRRenderTarget===!0),Ge=Me(R);i.renderbufferStorageMultisample(i.RENDERBUFFER,Ge,ye,R.width,R.height),i.framebufferRenderbuffer(i.FRAMEBUFFER,i.COLOR_ATTACHMENT0+de,i.RENDERBUFFER,ee.__webglColorRenderbuffer[de])}i.bindRenderbuffer(i.RENDERBUFFER,null),R.depthBuffer&&(ee.__webglDepthRenderbuffer=i.createRenderbuffer(),j(ee.__webglDepthRenderbuffer,R,!0)),t.bindFramebuffer(i.FRAMEBUFFER,null)}}if(oe){t.bindTexture(i.TEXTURE_CUBE_MAP,re.__webglTexture),N(i.TEXTURE_CUBE_MAP,y);for(let de=0;de<6;de++)if(y.mipmaps&&y.mipmaps.length>0)for(let he=0;he<y.mipmaps.length;he++)B(ee.__webglFramebuffer[de][he],R,y,i.COLOR_ATTACHMENT0,i.TEXTURE_CUBE_MAP_POSITIVE_X+de,he);else B(ee.__webglFramebuffer[de],R,y,i.COLOR_ATTACHMENT0,i.TEXTURE_CUBE_MAP_POSITIVE_X+de,0);m(y)&&p(i.TEXTURE_CUBE_MAP),t.unbindTexture()}else if(be){for(let de=0,he=ae.length;de<he;de++){const Ne=ae[de],ce=n.get(Ne);t.bindTexture(i.TEXTURE_2D,ce.__webglTexture),N(i.TEXTURE_2D,Ne),B(ee.__webglFramebuffer,R,Ne,i.COLOR_ATTACHMENT0+de,i.TEXTURE_2D,0),m(Ne)&&p(i.TEXTURE_2D)}t.unbindTexture()}else{let de=i.TEXTURE_2D;if((R.isWebGL3DRenderTarget||R.isWebGLArrayRenderTarget)&&(de=R.isWebGL3DRenderTarget?i.TEXTURE_3D:i.TEXTURE_2D_ARRAY),t.bindTexture(de,re.__webglTexture),N(de,y),y.mipmaps&&y.mipmaps.length>0)for(let he=0;he<y.mipmaps.length;he++)B(ee.__webglFramebuffer[he],R,y,i.COLOR_ATTACHMENT0,de,he);else B(ee.__webglFramebuffer,R,y,i.COLOR_ATTACHMENT0,de,0);m(y)&&p(de),t.unbindTexture()}R.depthBuffer&&le(R)}function L(R){const y=R.textures;for(let ee=0,re=y.length;ee<re;ee++){const ae=y[ee];if(m(ae)){const oe=R.isWebGLCubeRenderTarget?i.TEXTURE_CUBE_MAP:i.TEXTURE_2D,be=n.get(ae).__webglTexture;t.bindTexture(oe,be),p(oe),t.unbindTexture()}}}const xe=[],we=[];function Xe(R){if(R.samples>0){if(Be(R)===!1){const y=R.textures,ee=R.width,re=R.height;let ae=i.COLOR_BUFFER_BIT;const oe=R.stencilBuffer?i.DEPTH_STENCIL_ATTACHMENT:i.DEPTH_ATTACHMENT,be=n.get(R),de=y.length>1;if(de)for(let he=0;he<y.length;he++)t.bindFramebuffer(i.FRAMEBUFFER,be.__webglMultisampledFramebuffer),i.framebufferRenderbuffer(i.FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.RENDERBUFFER,null),t.bindFramebuffer(i.FRAMEBUFFER,be.__webglFramebuffer),i.framebufferTexture2D(i.DRAW_FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.TEXTURE_2D,null,0);t.bindFramebuffer(i.READ_FRAMEBUFFER,be.__webglMultisampledFramebuffer),t.bindFramebuffer(i.DRAW_FRAMEBUFFER,be.__webglFramebuffer);for(let he=0;he<y.length;he++){if(R.resolveDepthBuffer&&(R.depthBuffer&&(ae|=i.DEPTH_BUFFER_BIT),R.stencilBuffer&&R.resolveStencilBuffer&&(ae|=i.STENCIL_BUFFER_BIT)),de){i.framebufferRenderbuffer(i.READ_FRAMEBUFFER,i.COLOR_ATTACHMENT0,i.RENDERBUFFER,be.__webglColorRenderbuffer[he]);const Ne=n.get(y[he]).__webglTexture;i.framebufferTexture2D(i.DRAW_FRAMEBUFFER,i.COLOR_ATTACHMENT0,i.TEXTURE_2D,Ne,0)}i.blitFramebuffer(0,0,ee,re,0,0,ee,re,ae,i.NEAREST),l===!0&&(xe.length=0,we.length=0,xe.push(i.COLOR_ATTACHMENT0+he),R.depthBuffer&&R.resolveDepthBuffer===!1&&(xe.push(oe),we.push(oe),i.invalidateFramebuffer(i.DRAW_FRAMEBUFFER,we)),i.invalidateFramebuffer(i.READ_FRAMEBUFFER,xe))}if(t.bindFramebuffer(i.READ_FRAMEBUFFER,null),t.bindFramebuffer(i.DRAW_FRAMEBUFFER,null),de)for(let he=0;he<y.length;he++){t.bindFramebuffer(i.FRAMEBUFFER,be.__webglMultisampledFramebuffer),i.framebufferRenderbuffer(i.FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.RENDERBUFFER,be.__webglColorRenderbuffer[he]);const Ne=n.get(y[he]).__webglTexture;t.bindFramebuffer(i.FRAMEBUFFER,be.__webglFramebuffer),i.framebufferTexture2D(i.DRAW_FRAMEBUFFER,i.COLOR_ATTACHMENT0+he,i.TEXTURE_2D,Ne,0)}t.bindFramebuffer(i.DRAW_FRAMEBUFFER,be.__webglMultisampledFramebuffer)}else if(R.depthBuffer&&R.resolveDepthBuffer===!1&&l){const y=R.stencilBuffer?i.DEPTH_STENCIL_ATTACHMENT:i.DEPTH_ATTACHMENT;i.invalidateFramebuffer(i.DRAW_FRAMEBUFFER,[y])}}}function Me(R){return Math.min(r.maxSamples,R.samples)}function Be(R){const y=n.get(R);return R.samples>0&&e.has("WEBGL_multisampled_render_to_texture")===!0&&y.__useRenderToTexture!==!1}function Ie(R){const y=o.render.frame;u.get(R)!==y&&(u.set(R,y),R.update())}function Ae(R,y){const ee=R.colorSpace,re=R.format,ae=R.type;return R.isCompressedTexture===!0||R.isVideoTexture===!0||ee!==_n&&ee!==cn&&(je.getTransfer(ee)===Ke?(re!==zt||ae!==mn)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",ee)),y}function Ve(R){return typeof HTMLImageElement<"u"&&R instanceof HTMLImageElement?(c.width=R.naturalWidth||R.width,c.height=R.naturalHeight||R.height):typeof VideoFrame<"u"&&R instanceof VideoFrame?(c.width=R.displayWidth,c.height=R.displayHeight):(c.width=R.width,c.height=R.height),c}this.allocateTextureUnit=H,this.resetTextureUnits=C,this.setTexture2D=$,this.setTexture2DArray=z,this.setTexture3D=q,this.setTextureCube=te,this.rebindTextures=ve,this.setupRenderTarget=ge,this.updateRenderTargetMipmap=L,this.updateMultisampleRenderTarget=Xe,this.setupDepthRenderbuffer=le,this.setupFrameBufferTexture=B,this.useMultisampledRTT=Be}function z_(i,e){function t(n,r=cn){let s;const o=je.getTransfer(r);if(n===mn)return i.UNSIGNED_BYTE;if(n===Lc)return i.UNSIGNED_SHORT_4_4_4_4;if(n===Uc)return i.UNSIGNED_SHORT_5_5_5_1;if(n===Sd)return i.UNSIGNED_INT_5_9_9_9_REV;if(n===xd)return i.BYTE;if(n===yd)return i.SHORT;if(n===fr)return i.UNSIGNED_SHORT;if(n===Pc)return i.INT;if(n===ui)return i.UNSIGNED_INT;if(n===un)return i.FLOAT;if(n===br)return i.HALF_FLOAT;if(n===Md)return i.ALPHA;if(n===bd)return i.RGB;if(n===zt)return i.RGBA;if(n===Ed)return i.LUMINANCE;if(n===wd)return i.LUMINANCE_ALPHA;if(n===ai)return i.DEPTH_COMPONENT;if(n===di)return i.DEPTH_STENCIL;if(n===Td)return i.RED;if(n===Dc)return i.RED_INTEGER;if(n===Ad)return i.RG;if(n===Ic)return i.RG_INTEGER;if(n===Nc)return i.RGBA_INTEGER;if(n===Es||n===ws||n===Ts||n===As)if(o===Ke)if(s=e.get("WEBGL_compressed_texture_s3tc_srgb"),s!==null){if(n===Es)return s.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(n===ws)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(n===Ts)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(n===As)return s.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(s=e.get("WEBGL_compressed_texture_s3tc"),s!==null){if(n===Es)return s.COMPRESSED_RGB_S3TC_DXT1_EXT;if(n===ws)return s.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(n===Ts)return s.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(n===As)return s.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(n===Po||n===Lo||n===Uo||n===Do)if(s=e.get("WEBGL_compressed_texture_pvrtc"),s!==null){if(n===Po)return s.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(n===Lo)return s.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(n===Uo)return s.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(n===Do)return s.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(n===Io||n===No||n===Oo)if(s=e.get("WEBGL_compressed_texture_etc"),s!==null){if(n===Io||n===No)return o===Ke?s.COMPRESSED_SRGB8_ETC2:s.COMPRESSED_RGB8_ETC2;if(n===Oo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:s.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(n===Fo||n===Bo||n===ko||n===zo||n===Vo||n===Go||n===Ho||n===Wo||n===Xo||n===qo||n===Yo||n===jo||n===$o||n===Ko)if(s=e.get("WEBGL_compressed_texture_astc"),s!==null){if(n===Fo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:s.COMPRESSED_RGBA_ASTC_4x4_KHR;if(n===Bo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:s.COMPRESSED_RGBA_ASTC_5x4_KHR;if(n===ko)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:s.COMPRESSED_RGBA_ASTC_5x5_KHR;if(n===zo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:s.COMPRESSED_RGBA_ASTC_6x5_KHR;if(n===Vo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:s.COMPRESSED_RGBA_ASTC_6x6_KHR;if(n===Go)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:s.COMPRESSED_RGBA_ASTC_8x5_KHR;if(n===Ho)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:s.COMPRESSED_RGBA_ASTC_8x6_KHR;if(n===Wo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:s.COMPRESSED_RGBA_ASTC_8x8_KHR;if(n===Xo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:s.COMPRESSED_RGBA_ASTC_10x5_KHR;if(n===qo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:s.COMPRESSED_RGBA_ASTC_10x6_KHR;if(n===Yo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:s.COMPRESSED_RGBA_ASTC_10x8_KHR;if(n===jo)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:s.COMPRESSED_RGBA_ASTC_10x10_KHR;if(n===$o)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:s.COMPRESSED_RGBA_ASTC_12x10_KHR;if(n===Ko)return o===Ke?s.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:s.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(n===Cs||n===Zo||n===Jo)if(s=e.get("EXT_texture_compression_bptc"),s!==null){if(n===Cs)return o===Ke?s.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:s.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(n===Zo)return s.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(n===Jo)return s.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(n===Cd||n===Qo||n===el||n===tl)if(s=e.get("EXT_texture_compression_rgtc"),s!==null){if(n===Cs)return s.COMPRESSED_RED_RGTC1_EXT;if(n===Qo)return s.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(n===el)return s.COMPRESSED_RED_GREEN_RGTC2_EXT;if(n===tl)return s.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return n===hi?i.UNSIGNED_INT_24_8:i[n]!==void 0?i[n]:null}return{convert:t}}class V_ extends Rt{constructor(e=[]){super(),this.isArrayCamera=!0,this.cameras=e}}class hn extends dt{constructor(){super(),this.isGroup=!0,this.type="Group"}}const G_={type:"move"};class ea{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new hn,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new hn,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new G,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new G),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new hn,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new G,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new G),this._grip}dispatchEvent(e){return this._targetRay!==null&&this._targetRay.dispatchEvent(e),this._grip!==null&&this._grip.dispatchEvent(e),this._hand!==null&&this._hand.dispatchEvent(e),this}connect(e){if(e&&e.hand){const t=this._hand;if(t)for(const n of e.hand.values())this._getHandJoint(t,n)}return this.dispatchEvent({type:"connected",data:e}),this}disconnect(e){return this.dispatchEvent({type:"disconnected",data:e}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(e,t,n){let r=null,s=null,o=null;const a=this._targetRay,l=this._grip,c=this._hand;if(e&&t.session.visibilityState!=="visible-blurred"){if(c&&e.hand){o=!0;for(const x of e.hand.values()){const m=t.getJointPose(x,n),p=this._getHandJoint(c,x);m!==null&&(p.matrix.fromArray(m.transform.matrix),p.matrix.decompose(p.position,p.rotation,p.scale),p.matrixWorldNeedsUpdate=!0,p.jointRadius=m.radius),p.visible=m!==null}const u=c.joints["index-finger-tip"],h=c.joints["thumb-tip"],d=u.position.distanceTo(h.position),f=.02,g=.005;c.inputState.pinching&&d>f+g?(c.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:e.handedness,target:this})):!c.inputState.pinching&&d<=f-g&&(c.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:e.handedness,target:this}))}else l!==null&&e.gripSpace&&(s=t.getPose(e.gripSpace,n),s!==null&&(l.matrix.fromArray(s.transform.matrix),l.matrix.decompose(l.position,l.rotation,l.scale),l.matrixWorldNeedsUpdate=!0,s.linearVelocity?(l.hasLinearVelocity=!0,l.linearVelocity.copy(s.linearVelocity)):l.hasLinearVelocity=!1,s.angularVelocity?(l.hasAngularVelocity=!0,l.angularVelocity.copy(s.angularVelocity)):l.hasAngularVelocity=!1));a!==null&&(r=t.getPose(e.targetRaySpace,n),r===null&&s!==null&&(r=s),r!==null&&(a.matrix.fromArray(r.transform.matrix),a.matrix.decompose(a.position,a.rotation,a.scale),a.matrixWorldNeedsUpdate=!0,r.linearVelocity?(a.hasLinearVelocity=!0,a.linearVelocity.copy(r.linearVelocity)):a.hasLinearVelocity=!1,r.angularVelocity?(a.hasAngularVelocity=!0,a.angularVelocity.copy(r.angularVelocity)):a.hasAngularVelocity=!1,this.dispatchEvent(G_)))}return a!==null&&(a.visible=r!==null),l!==null&&(l.visible=s!==null),c!==null&&(c.visible=o!==null),this}_getHandJoint(e,t){if(e.joints[t.jointName]===void 0){const n=new hn;n.matrixAutoUpdate=!1,n.visible=!1,e.joints[t.jointName]=n,e.add(n)}return e.joints[t.jointName]}}const H_=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,W_=`
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

}`;class X_{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(e,t,n){if(this.texture===null){const r=new St,s=e.properties.get(r);s.__webglTexture=t.texture,(t.depthNear!=n.depthNear||t.depthFar!=n.depthFar)&&(this.depthNear=t.depthNear,this.depthFar=t.depthFar),this.texture=r}}getMesh(e){if(this.texture!==null&&this.mesh===null){const t=e.cameras[0].viewport,n=new gn({vertexShader:H_,fragmentShader:W_,uniforms:{depthColor:{value:this.texture},depthWidth:{value:t.z},depthHeight:{value:t.w}}});this.mesh=new Ot(new Tr(20,20),n)}return this.mesh}reset(){this.texture=null,this.mesh=null}}class q_ extends Fn{constructor(e,t){super();const n=this;let r=null,s=1,o=null,a="local-floor",l=1,c=null,u=null,h=null,d=null,f=null,g=null;const x=new X_,m=t.getContextAttributes();let p=null,w=null;const S=[],A=[],k=new De;let I=null;const P=new Rt;P.layers.enable(1),P.viewport=new ht;const Y=new Rt;Y.layers.enable(2),Y.viewport=new ht;const E=[P,Y],b=new V_;b.layers.enable(1),b.layers.enable(2);let C=null,H=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(D){let B=S[D];return B===void 0&&(B=new ea,S[D]=B),B.getTargetRaySpace()},this.getControllerGrip=function(D){let B=S[D];return B===void 0&&(B=new ea,S[D]=B),B.getGripSpace()},this.getHand=function(D){let B=S[D];return B===void 0&&(B=new ea,S[D]=B),B.getHandSpace()};function O(D){const B=A.indexOf(D.inputSource);if(B===-1)return;const j=S[B];j!==void 0&&(j.update(D.inputSource,D.frame,c||o),j.dispatchEvent({type:D.type,data:D.inputSource}))}function $(){r.removeEventListener("select",O),r.removeEventListener("selectstart",O),r.removeEventListener("selectend",O),r.removeEventListener("squeeze",O),r.removeEventListener("squeezestart",O),r.removeEventListener("squeezeend",O),r.removeEventListener("end",$),r.removeEventListener("inputsourceschange",z);for(let D=0;D<S.length;D++){const B=A[D];B!==null&&(A[D]=null,S[D].disconnect(B))}C=null,H=null,x.reset(),e.setRenderTarget(p),f=null,d=null,h=null,r=null,w=null,K.stop(),n.isPresenting=!1,e.setPixelRatio(I),e.setSize(k.width,k.height,!1),n.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(D){s=D,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(D){a=D,n.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return c||o},this.setReferenceSpace=function(D){c=D},this.getBaseLayer=function(){return d!==null?d:f},this.getBinding=function(){return h},this.getFrame=function(){return g},this.getSession=function(){return r},this.setSession=async function(D){if(r=D,r!==null){if(p=e.getRenderTarget(),r.addEventListener("select",O),r.addEventListener("selectstart",O),r.addEventListener("selectend",O),r.addEventListener("squeeze",O),r.addEventListener("squeezestart",O),r.addEventListener("squeezeend",O),r.addEventListener("end",$),r.addEventListener("inputsourceschange",z),m.xrCompatible!==!0&&await t.makeXRCompatible(),I=e.getPixelRatio(),e.getSize(k),r.renderState.layers===void 0){const B={antialias:m.antialias,alpha:!0,depth:m.depth,stencil:m.stencil,framebufferScaleFactor:s};f=new XRWebGLLayer(r,t,B),r.updateRenderState({baseLayer:f}),e.setPixelRatio(1),e.setSize(f.framebufferWidth,f.framebufferHeight,!1),w=new Un(f.framebufferWidth,f.framebufferHeight,{format:zt,type:mn,colorSpace:e.outputColorSpace,stencilBuffer:m.stencil})}else{let B=null,j=null,X=null;m.depth&&(X=m.stencil?t.DEPTH24_STENCIL8:t.DEPTH_COMPONENT24,B=m.stencil?di:ai,j=m.stencil?hi:ui);const le={colorFormat:t.RGBA8,depthFormat:X,scaleFactor:s};h=new XRWebGLBinding(r,t),d=h.createProjectionLayer(le),r.updateRenderState({layers:[d]}),e.setPixelRatio(1),e.setSize(d.textureWidth,d.textureHeight,!1),w=new Un(d.textureWidth,d.textureHeight,{format:zt,type:mn,depthTexture:new Jc(d.textureWidth,d.textureHeight,j,void 0,void 0,void 0,void 0,void 0,void 0,B),stencilBuffer:m.stencil,colorSpace:e.outputColorSpace,samples:m.antialias?4:0,resolveDepthBuffer:d.ignoreDepthValues===!1})}w.isXRRenderTarget=!0,this.setFoveation(l),c=null,o=await r.requestReferenceSpace(a),K.setContext(r),K.start(),n.isPresenting=!0,n.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(r!==null)return r.environmentBlendMode};function z(D){for(let B=0;B<D.removed.length;B++){const j=D.removed[B],X=A.indexOf(j);X>=0&&(A[X]=null,S[X].disconnect(j))}for(let B=0;B<D.added.length;B++){const j=D.added[B];let X=A.indexOf(j);if(X===-1){for(let ve=0;ve<S.length;ve++)if(ve>=A.length){A.push(j),X=ve;break}else if(A[ve]===null){A[ve]=j,X=ve;break}if(X===-1)break}const le=S[X];le&&le.connect(j)}}const q=new G,te=new G;function v(D,B,j){q.setFromMatrixPosition(B.matrixWorld),te.setFromMatrixPosition(j.matrixWorld);const X=q.distanceTo(te),le=B.projectionMatrix.elements,ve=j.projectionMatrix.elements,ge=le[14]/(le[10]-1),L=le[14]/(le[10]+1),xe=(le[9]+1)/le[5],we=(le[9]-1)/le[5],Xe=(le[8]-1)/le[0],Me=(ve[8]+1)/ve[0],Be=ge*Xe,Ie=ge*Me,Ae=X/(-Xe+Me),Ve=Ae*-Xe;B.matrixWorld.decompose(D.position,D.quaternion,D.scale),D.translateX(Ve),D.translateZ(Ae),D.matrixWorld.compose(D.position,D.quaternion,D.scale),D.matrixWorldInverse.copy(D.matrixWorld).invert();const R=ge+Ae,y=L+Ae,ee=Be-Ve,re=Ie+(X-Ve),ae=xe*L/y*R,oe=we*L/y*R;D.projectionMatrix.makePerspective(ee,re,ae,oe,R,y),D.projectionMatrixInverse.copy(D.projectionMatrix).invert()}function T(D,B){B===null?D.matrixWorld.copy(D.matrix):D.matrixWorld.multiplyMatrices(B.matrixWorld,D.matrix),D.matrixWorldInverse.copy(D.matrixWorld).invert()}this.updateCamera=function(D){if(r===null)return;x.texture!==null&&(D.near=x.depthNear,D.far=x.depthFar),b.near=Y.near=P.near=D.near,b.far=Y.far=P.far=D.far,(C!==b.near||H!==b.far)&&(r.updateRenderState({depthNear:b.near,depthFar:b.far}),C=b.near,H=b.far,P.near=C,P.far=H,Y.near=C,Y.far=H,P.updateProjectionMatrix(),Y.updateProjectionMatrix(),D.updateProjectionMatrix());const B=D.parent,j=b.cameras;T(b,B);for(let X=0;X<j.length;X++)T(j[X],B);j.length===2?v(b,P,Y):b.projectionMatrix.copy(P.projectionMatrix),U(D,b,B)};function U(D,B,j){j===null?D.matrix.copy(B.matrixWorld):(D.matrix.copy(j.matrixWorld),D.matrix.invert(),D.matrix.multiply(B.matrixWorld)),D.matrix.decompose(D.position,D.quaternion,D.scale),D.updateMatrixWorld(!0),D.projectionMatrix.copy(B.projectionMatrix),D.projectionMatrixInverse.copy(B.projectionMatrixInverse),D.isPerspectiveCamera&&(D.fov=ha*2*Math.atan(1/D.projectionMatrix.elements[5]),D.zoom=1)}this.getCamera=function(){return b},this.getFoveation=function(){if(!(d===null&&f===null))return l},this.setFoveation=function(D){l=D,d!==null&&(d.fixedFoveation=D),f!==null&&f.fixedFoveation!==void 0&&(f.fixedFoveation=D)},this.hasDepthSensing=function(){return x.texture!==null},this.getDepthSensingMesh=function(){return x.getMesh(b)};let N=null;function V(D,B){if(u=B.getViewerPose(c||o),g=B,u!==null){const j=u.views;f!==null&&(e.setRenderTargetFramebuffer(w,f.framebuffer),e.setRenderTarget(w));let X=!1;j.length!==b.cameras.length&&(b.cameras.length=0,X=!0);for(let ve=0;ve<j.length;ve++){const ge=j[ve];let L=null;if(f!==null)L=f.getViewport(ge);else{const we=h.getViewSubImage(d,ge);L=we.viewport,ve===0&&(e.setRenderTargetTextures(w,we.colorTexture,d.ignoreDepthValues?void 0:we.depthStencilTexture),e.setRenderTarget(w))}let xe=E[ve];xe===void 0&&(xe=new Rt,xe.layers.enable(ve),xe.viewport=new ht,E[ve]=xe),xe.matrix.fromArray(ge.transform.matrix),xe.matrix.decompose(xe.position,xe.quaternion,xe.scale),xe.projectionMatrix.fromArray(ge.projectionMatrix),xe.projectionMatrixInverse.copy(xe.projectionMatrix).invert(),xe.viewport.set(L.x,L.y,L.width,L.height),ve===0&&(b.matrix.copy(xe.matrix),b.matrix.decompose(b.position,b.quaternion,b.scale)),X===!0&&b.cameras.push(xe)}const le=r.enabledFeatures;if(le&&le.includes("depth-sensing")){const ve=h.getDepthInformation(j[0]);ve&&ve.isValid&&ve.texture&&x.init(e,ve,r.renderState)}}for(let j=0;j<S.length;j++){const X=A[j],le=S[j];X!==null&&le!==void 0&&le.update(X,B,c||o)}N&&N(D,B),B.detectedPlanes&&n.dispatchEvent({type:"planesdetected",data:B}),g=null}const K=new Kc;K.setAnimationLoop(V),this.setAnimationLoop=function(D){N=D},this.dispose=function(){}}}const Tn=new Ht,Y_=new Je;function j_(i,e){function t(m,p){m.matrixAutoUpdate===!0&&m.updateMatrix(),p.value.copy(m.matrix)}function n(m,p){p.color.getRGB(m.fogColor.value,Yc(i)),p.isFog?(m.fogNear.value=p.near,m.fogFar.value=p.far):p.isFogExp2&&(m.fogDensity.value=p.density)}function r(m,p,w,S,A){p.isMeshBasicMaterial||p.isMeshLambertMaterial?s(m,p):p.isMeshToonMaterial?(s(m,p),h(m,p)):p.isMeshPhongMaterial?(s(m,p),u(m,p)):p.isMeshStandardMaterial?(s(m,p),d(m,p),p.isMeshPhysicalMaterial&&f(m,p,A)):p.isMeshMatcapMaterial?(s(m,p),g(m,p)):p.isMeshDepthMaterial?s(m,p):p.isMeshDistanceMaterial?(s(m,p),x(m,p)):p.isMeshNormalMaterial?s(m,p):p.isLineBasicMaterial?(o(m,p),p.isLineDashedMaterial&&a(m,p)):p.isPointsMaterial?l(m,p,w,S):p.isSpriteMaterial?c(m,p):p.isShadowMaterial?(m.color.value.copy(p.color),m.opacity.value=p.opacity):p.isShaderMaterial&&(p.uniformsNeedUpdate=!1)}function s(m,p){m.opacity.value=p.opacity,p.color&&m.diffuse.value.copy(p.color),p.emissive&&m.emissive.value.copy(p.emissive).multiplyScalar(p.emissiveIntensity),p.map&&(m.map.value=p.map,t(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.bumpMap&&(m.bumpMap.value=p.bumpMap,t(p.bumpMap,m.bumpMapTransform),m.bumpScale.value=p.bumpScale,p.side===yt&&(m.bumpScale.value*=-1)),p.normalMap&&(m.normalMap.value=p.normalMap,t(p.normalMap,m.normalMapTransform),m.normalScale.value.copy(p.normalScale),p.side===yt&&m.normalScale.value.negate()),p.displacementMap&&(m.displacementMap.value=p.displacementMap,t(p.displacementMap,m.displacementMapTransform),m.displacementScale.value=p.displacementScale,m.displacementBias.value=p.displacementBias),p.emissiveMap&&(m.emissiveMap.value=p.emissiveMap,t(p.emissiveMap,m.emissiveMapTransform)),p.specularMap&&(m.specularMap.value=p.specularMap,t(p.specularMap,m.specularMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest);const w=e.get(p),S=w.envMap,A=w.envMapRotation;S&&(m.envMap.value=S,Tn.copy(A),Tn.x*=-1,Tn.y*=-1,Tn.z*=-1,S.isCubeTexture&&S.isRenderTargetTexture===!1&&(Tn.y*=-1,Tn.z*=-1),m.envMapRotation.value.setFromMatrix4(Y_.makeRotationFromEuler(Tn)),m.flipEnvMap.value=S.isCubeTexture&&S.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=p.reflectivity,m.ior.value=p.ior,m.refractionRatio.value=p.refractionRatio),p.lightMap&&(m.lightMap.value=p.lightMap,m.lightMapIntensity.value=p.lightMapIntensity,t(p.lightMap,m.lightMapTransform)),p.aoMap&&(m.aoMap.value=p.aoMap,m.aoMapIntensity.value=p.aoMapIntensity,t(p.aoMap,m.aoMapTransform))}function o(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,p.map&&(m.map.value=p.map,t(p.map,m.mapTransform))}function a(m,p){m.dashSize.value=p.dashSize,m.totalSize.value=p.dashSize+p.gapSize,m.scale.value=p.scale}function l(m,p,w,S){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.size.value=p.size*w,m.scale.value=S*.5,p.map&&(m.map.value=p.map,t(p.map,m.uvTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function c(m,p){m.diffuse.value.copy(p.color),m.opacity.value=p.opacity,m.rotation.value=p.rotation,p.map&&(m.map.value=p.map,t(p.map,m.mapTransform)),p.alphaMap&&(m.alphaMap.value=p.alphaMap,t(p.alphaMap,m.alphaMapTransform)),p.alphaTest>0&&(m.alphaTest.value=p.alphaTest)}function u(m,p){m.specular.value.copy(p.specular),m.shininess.value=Math.max(p.shininess,1e-4)}function h(m,p){p.gradientMap&&(m.gradientMap.value=p.gradientMap)}function d(m,p){m.metalness.value=p.metalness,p.metalnessMap&&(m.metalnessMap.value=p.metalnessMap,t(p.metalnessMap,m.metalnessMapTransform)),m.roughness.value=p.roughness,p.roughnessMap&&(m.roughnessMap.value=p.roughnessMap,t(p.roughnessMap,m.roughnessMapTransform)),p.envMap&&(m.envMapIntensity.value=p.envMapIntensity)}function f(m,p,w){m.ior.value=p.ior,p.sheen>0&&(m.sheenColor.value.copy(p.sheenColor).multiplyScalar(p.sheen),m.sheenRoughness.value=p.sheenRoughness,p.sheenColorMap&&(m.sheenColorMap.value=p.sheenColorMap,t(p.sheenColorMap,m.sheenColorMapTransform)),p.sheenRoughnessMap&&(m.sheenRoughnessMap.value=p.sheenRoughnessMap,t(p.sheenRoughnessMap,m.sheenRoughnessMapTransform))),p.clearcoat>0&&(m.clearcoat.value=p.clearcoat,m.clearcoatRoughness.value=p.clearcoatRoughness,p.clearcoatMap&&(m.clearcoatMap.value=p.clearcoatMap,t(p.clearcoatMap,m.clearcoatMapTransform)),p.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=p.clearcoatRoughnessMap,t(p.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),p.clearcoatNormalMap&&(m.clearcoatNormalMap.value=p.clearcoatNormalMap,t(p.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(p.clearcoatNormalScale),p.side===yt&&m.clearcoatNormalScale.value.negate())),p.dispersion>0&&(m.dispersion.value=p.dispersion),p.iridescence>0&&(m.iridescence.value=p.iridescence,m.iridescenceIOR.value=p.iridescenceIOR,m.iridescenceThicknessMinimum.value=p.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=p.iridescenceThicknessRange[1],p.iridescenceMap&&(m.iridescenceMap.value=p.iridescenceMap,t(p.iridescenceMap,m.iridescenceMapTransform)),p.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=p.iridescenceThicknessMap,t(p.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),p.transmission>0&&(m.transmission.value=p.transmission,m.transmissionSamplerMap.value=w.texture,m.transmissionSamplerSize.value.set(w.width,w.height),p.transmissionMap&&(m.transmissionMap.value=p.transmissionMap,t(p.transmissionMap,m.transmissionMapTransform)),m.thickness.value=p.thickness,p.thicknessMap&&(m.thicknessMap.value=p.thicknessMap,t(p.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=p.attenuationDistance,m.attenuationColor.value.copy(p.attenuationColor)),p.anisotropy>0&&(m.anisotropyVector.value.set(p.anisotropy*Math.cos(p.anisotropyRotation),p.anisotropy*Math.sin(p.anisotropyRotation)),p.anisotropyMap&&(m.anisotropyMap.value=p.anisotropyMap,t(p.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=p.specularIntensity,m.specularColor.value.copy(p.specularColor),p.specularColorMap&&(m.specularColorMap.value=p.specularColorMap,t(p.specularColorMap,m.specularColorMapTransform)),p.specularIntensityMap&&(m.specularIntensityMap.value=p.specularIntensityMap,t(p.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,p){p.matcap&&(m.matcap.value=p.matcap)}function x(m,p){const w=e.get(p).light;m.referencePosition.value.setFromMatrixPosition(w.matrixWorld),m.nearDistance.value=w.shadow.camera.near,m.farDistance.value=w.shadow.camera.far}return{refreshFogUniforms:n,refreshMaterialUniforms:r}}function $_(i,e,t,n){let r={},s={},o=[];const a=i.getParameter(i.MAX_UNIFORM_BUFFER_BINDINGS);function l(w,S){const A=S.program;n.uniformBlockBinding(w,A)}function c(w,S){let A=r[w.id];A===void 0&&(g(w),A=u(w),r[w.id]=A,w.addEventListener("dispose",m));const k=S.program;n.updateUBOMapping(w,k);const I=e.render.frame;s[w.id]!==I&&(d(w),s[w.id]=I)}function u(w){const S=h();w.__bindingPointIndex=S;const A=i.createBuffer(),k=w.__size,I=w.usage;return i.bindBuffer(i.UNIFORM_BUFFER,A),i.bufferData(i.UNIFORM_BUFFER,k,I),i.bindBuffer(i.UNIFORM_BUFFER,null),i.bindBufferBase(i.UNIFORM_BUFFER,S,A),A}function h(){for(let w=0;w<a;w++)if(o.indexOf(w)===-1)return o.push(w),w;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(w){const S=r[w.id],A=w.uniforms,k=w.__cache;i.bindBuffer(i.UNIFORM_BUFFER,S);for(let I=0,P=A.length;I<P;I++){const Y=Array.isArray(A[I])?A[I]:[A[I]];for(let E=0,b=Y.length;E<b;E++){const C=Y[E];if(f(C,I,E,k)===!0){const H=C.__offset,O=Array.isArray(C.value)?C.value:[C.value];let $=0;for(let z=0;z<O.length;z++){const q=O[z],te=x(q);typeof q=="number"||typeof q=="boolean"?(C.__data[0]=q,i.bufferSubData(i.UNIFORM_BUFFER,H+$,C.__data)):q.isMatrix3?(C.__data[0]=q.elements[0],C.__data[1]=q.elements[1],C.__data[2]=q.elements[2],C.__data[3]=0,C.__data[4]=q.elements[3],C.__data[5]=q.elements[4],C.__data[6]=q.elements[5],C.__data[7]=0,C.__data[8]=q.elements[6],C.__data[9]=q.elements[7],C.__data[10]=q.elements[8],C.__data[11]=0):(q.toArray(C.__data,$),$+=te.storage/Float32Array.BYTES_PER_ELEMENT)}i.bufferSubData(i.UNIFORM_BUFFER,H,C.__data)}}}i.bindBuffer(i.UNIFORM_BUFFER,null)}function f(w,S,A,k){const I=w.value,P=S+"_"+A;if(k[P]===void 0)return typeof I=="number"||typeof I=="boolean"?k[P]=I:k[P]=I.clone(),!0;{const Y=k[P];if(typeof I=="number"||typeof I=="boolean"){if(Y!==I)return k[P]=I,!0}else if(Y.equals(I)===!1)return Y.copy(I),!0}return!1}function g(w){const S=w.uniforms;let A=0;const k=16;for(let P=0,Y=S.length;P<Y;P++){const E=Array.isArray(S[P])?S[P]:[S[P]];for(let b=0,C=E.length;b<C;b++){const H=E[b],O=Array.isArray(H.value)?H.value:[H.value];for(let $=0,z=O.length;$<z;$++){const q=O[$],te=x(q),v=A%k;v!==0&&k-v<te.boundary&&(A+=k-v),H.__data=new Float32Array(te.storage/Float32Array.BYTES_PER_ELEMENT),H.__offset=A,A+=te.storage}}}const I=A%k;return I>0&&(A+=k-I),w.__size=A,w.__cache={},this}function x(w){const S={boundary:0,storage:0};return typeof w=="number"||typeof w=="boolean"?(S.boundary=4,S.storage=4):w.isVector2?(S.boundary=8,S.storage=8):w.isVector3||w.isColor?(S.boundary=16,S.storage=12):w.isVector4?(S.boundary=16,S.storage=16):w.isMatrix3?(S.boundary=48,S.storage=48):w.isMatrix4?(S.boundary=64,S.storage=64):w.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",w),S}function m(w){const S=w.target;S.removeEventListener("dispose",m);const A=o.indexOf(S.__bindingPointIndex);o.splice(A,1),i.deleteBuffer(r[S.id]),delete r[S.id],delete s[S.id]}function p(){for(const w in r)i.deleteBuffer(r[w]);o=[],r={},s={}}return{bind:l,update:c,dispose:p}}class K_{constructor(e={}){const{canvas:t=Vd(),context:n=null,depth:r=!0,stencil:s=!1,alpha:o=!1,antialias:a=!1,premultipliedAlpha:l=!0,preserveDrawingBuffer:c=!1,powerPreference:u="default",failIfMajorPerformanceCaveat:h=!1}=e;this.isWebGLRenderer=!0;let d;if(n!==null){if(typeof WebGLRenderingContext<"u"&&n instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");d=n.getContextAttributes().alpha}else d=o;const f=new Uint32Array(4),g=new Int32Array(4);let x=null,m=null;const p=[],w=[];this.domElement=t,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=Ft,this.toneMapping=fn,this.toneMappingExposure=1;const S=this;let A=!1,k=0,I=0,P=null,Y=-1,E=null;const b=new ht,C=new ht;let H=null;const O=new He(0);let $=0,z=t.width,q=t.height,te=1,v=null,T=null;const U=new ht(0,0,z,q),N=new ht(0,0,z,q);let V=!1;const K=new Sa;let D=!1,B=!1;const j=new Je,X=new G,le={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};let ve=!1;function ge(){return P===null?te:1}let L=n;function xe(M,F){return t.getContext(M,F)}try{const M={alpha:!0,depth:r,stencil:s,antialias:a,premultipliedAlpha:l,preserveDrawingBuffer:c,powerPreference:u,failIfMajorPerformanceCaveat:h};if("setAttribute"in t&&t.setAttribute("data-engine",`three.js r${xa}`),t.addEventListener("webglcontextlost",ne,!1),t.addEventListener("webglcontextrestored",Z,!1),t.addEventListener("webglcontextcreationerror",ie,!1),L===null){const F="webgl2";if(L=xe(F,M),L===null)throw xe(F)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(M){throw console.error("THREE.WebGLRenderer: "+M.message),M}let we,Xe,Me,Be,Ie,Ae,Ve,R,y,ee,re,ae,oe,be,de,he,Ne,ce,ye,Ge,Re,pe,Oe,Fe;function Qe(){we=new rg(L),we.init(),pe=new z_(L,we),Xe=new Jm(L,we,e,pe),Me=new B_(L),Be=new og(L),Ie=new E_,Ae=new k_(L,we,Me,Ie,Xe,pe,Be),Ve=new eg(S),R=new ig(S),y=new pf(L),Oe=new Km(L,y),ee=new sg(L,y,Be,Oe),re=new cg(L,ee,y,Be),ye=new lg(L,Xe,Ae),he=new Qm(Ie),ae=new b_(S,Ve,R,we,Xe,Oe,he),oe=new j_(S,Ie),be=new T_,de=new U_(we),ce=new $m(S,Ve,R,Me,re,d,l),Ne=new F_(S,re,Xe),Fe=new $_(L,Be,Xe,Me),Ge=new Zm(L,we,Be),Re=new ag(L,we,Be),Be.programs=ae.programs,S.capabilities=Xe,S.extensions=we,S.properties=Ie,S.renderLists=be,S.shadowMap=Ne,S.state=Me,S.info=Be}Qe();const _=new q_(S,L);this.xr=_,this.getContext=function(){return L},this.getContextAttributes=function(){return L.getContextAttributes()},this.forceContextLoss=function(){const M=we.get("WEBGL_lose_context");M&&M.loseContext()},this.forceContextRestore=function(){const M=we.get("WEBGL_lose_context");M&&M.restoreContext()},this.getPixelRatio=function(){return te},this.setPixelRatio=function(M){M!==void 0&&(te=M,this.setSize(z,q,!1))},this.getSize=function(M){return M.set(z,q)},this.setSize=function(M,F,J=!0){if(_.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}z=M,q=F,t.width=Math.floor(M*te),t.height=Math.floor(F*te),J===!0&&(t.style.width=M+"px",t.style.height=F+"px"),this.setViewport(0,0,M,F)},this.getDrawingBufferSize=function(M){return M.set(z*te,q*te).floor()},this.setDrawingBufferSize=function(M,F,J){z=M,q=F,te=J,t.width=Math.floor(M*J),t.height=Math.floor(F*J),this.setViewport(0,0,M,F)},this.getCurrentViewport=function(M){return M.copy(b)},this.getViewport=function(M){return M.copy(U)},this.setViewport=function(M,F,J,Q){M.isVector4?U.set(M.x,M.y,M.z,M.w):U.set(M,F,J,Q),Me.viewport(b.copy(U).multiplyScalar(te).round())},this.getScissor=function(M){return M.copy(N)},this.setScissor=function(M,F,J,Q){M.isVector4?N.set(M.x,M.y,M.z,M.w):N.set(M,F,J,Q),Me.scissor(C.copy(N).multiplyScalar(te).round())},this.getScissorTest=function(){return V},this.setScissorTest=function(M){Me.setScissorTest(V=M)},this.setOpaqueSort=function(M){v=M},this.setTransparentSort=function(M){T=M},this.getClearColor=function(M){return M.copy(ce.getClearColor())},this.setClearColor=function(){ce.setClearColor.apply(ce,arguments)},this.getClearAlpha=function(){return ce.getClearAlpha()},this.setClearAlpha=function(){ce.setClearAlpha.apply(ce,arguments)},this.clear=function(M=!0,F=!0,J=!0){let Q=0;if(M){let W=!1;if(P!==null){const ue=P.texture.format;W=ue===Nc||ue===Ic||ue===Dc}if(W){const ue=P.texture.type,me=ue===mn||ue===ui||ue===fr||ue===hi||ue===Lc||ue===Uc,_e=ce.getClearColor(),Se=ce.getClearAlpha(),Pe=_e.r,Le=_e.g,Ce=_e.b;me?(f[0]=Pe,f[1]=Le,f[2]=Ce,f[3]=Se,L.clearBufferuiv(L.COLOR,0,f)):(g[0]=Pe,g[1]=Le,g[2]=Ce,g[3]=Se,L.clearBufferiv(L.COLOR,0,g))}else Q|=L.COLOR_BUFFER_BIT}F&&(Q|=L.DEPTH_BUFFER_BIT),J&&(Q|=L.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),L.clear(Q)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){t.removeEventListener("webglcontextlost",ne,!1),t.removeEventListener("webglcontextrestored",Z,!1),t.removeEventListener("webglcontextcreationerror",ie,!1),be.dispose(),de.dispose(),Ie.dispose(),Ve.dispose(),R.dispose(),re.dispose(),Oe.dispose(),Fe.dispose(),ae.dispose(),_.dispose(),_.removeEventListener("sessionstart",rt),_.removeEventListener("sessionend",st),Mt.stop()};function ne(M){M.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),A=!0}function Z(){console.log("THREE.WebGLRenderer: Context Restored."),A=!1;const M=Be.autoReset,F=Ne.enabled,J=Ne.autoUpdate,Q=Ne.needsUpdate,W=Ne.type;Qe(),Be.autoReset=M,Ne.enabled=F,Ne.autoUpdate=J,Ne.needsUpdate=Q,Ne.type=W}function ie(M){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",M.statusMessage)}function se(M){const F=M.target;F.removeEventListener("dispose",se),Ee(F)}function Ee(M){Ue(M),Ie.remove(M)}function Ue(M){const F=Ie.get(M).programs;F!==void 0&&(F.forEach(function(J){ae.releaseProgram(J)}),M.isShaderMaterial&&ae.releaseShaderCache(M))}this.renderBufferDirect=function(M,F,J,Q,W,ue){F===null&&(F=le);const me=W.isMesh&&W.matrixWorld.determinant()<0,_e=cu(M,F,J,Q,W);Me.setMaterial(Q,me);let Se=J.index,Pe=1;if(Q.wireframe===!0){if(Se=ee.getWireframeAttribute(J),Se===void 0)return;Pe=2}const Le=J.drawRange,Ce=J.attributes.position;let qe=Le.start*Pe,tt=(Le.start+Le.count)*Pe;ue!==null&&(qe=Math.max(qe,ue.start*Pe),tt=Math.min(tt,(ue.start+ue.count)*Pe)),Se!==null?(qe=Math.max(qe,0),tt=Math.min(tt,Se.count)):Ce!=null&&(qe=Math.max(qe,0),tt=Math.min(tt,Ce.count));const nt=tt-qe;if(nt<0||nt===1/0)return;Oe.setup(W,Q,_e,J,Se);let Et,Ye=Ge;if(Se!==null&&(Et=y.get(Se),Ye=Re,Ye.setIndex(Et)),W.isMesh)Q.wireframe===!0?(Me.setLineWidth(Q.wireframeLinewidth*ge()),Ye.setMode(L.LINES)):Ye.setMode(L.TRIANGLES);else if(W.isLine){let Te=Q.linewidth;Te===void 0&&(Te=1),Me.setLineWidth(Te*ge()),W.isLineSegments?Ye.setMode(L.LINES):W.isLineLoop?Ye.setMode(L.LINE_LOOP):Ye.setMode(L.LINE_STRIP)}else W.isPoints?Ye.setMode(L.POINTS):W.isSprite&&Ye.setMode(L.TRIANGLES);if(W.isBatchedMesh)W._multiDrawInstances!==null?Ye.renderMultiDrawInstances(W._multiDrawStarts,W._multiDrawCounts,W._multiDrawCount,W._multiDrawInstances):Ye.renderMultiDraw(W._multiDrawStarts,W._multiDrawCounts,W._multiDrawCount);else if(W.isInstancedMesh)Ye.renderInstances(qe,nt,W.count);else if(J.isInstancedBufferGeometry){const Te=J._maxInstanceCount!==void 0?J._maxInstanceCount:1/0,gt=Math.min(J.instanceCount,Te);Ye.renderInstances(qe,nt,gt)}else Ye.render(qe,nt)};function et(M,F,J){M.transparent===!0&&M.side===Zt&&M.forceSinglePass===!1?(M.side=yt,M.needsUpdate=!0,Di(M,F,J),M.side=pn,M.needsUpdate=!0,Di(M,F,J),M.side=Zt):Di(M,F,J)}this.compile=function(M,F,J=null){J===null&&(J=M),m=de.get(J),m.init(F),w.push(m),J.traverseVisible(function(W){W.isLight&&W.layers.test(F.layers)&&(m.pushLight(W),W.castShadow&&m.pushShadow(W))}),M!==J&&M.traverseVisible(function(W){W.isLight&&W.layers.test(F.layers)&&(m.pushLight(W),W.castShadow&&m.pushShadow(W))}),m.setupLights();const Q=new Set;return M.traverse(function(W){const ue=W.material;if(ue)if(Array.isArray(ue))for(let me=0;me<ue.length;me++){const _e=ue[me];et(_e,J,W),Q.add(_e)}else et(ue,J,W),Q.add(ue)}),w.pop(),m=null,Q},this.compileAsync=function(M,F,J=null){const Q=this.compile(M,F,J);return new Promise(W=>{function ue(){if(Q.forEach(function(me){Ie.get(me).currentProgram.isReady()&&Q.delete(me)}),Q.size===0){W(M);return}setTimeout(ue,10)}we.get("KHR_parallel_shader_compile")!==null?ue():setTimeout(ue,10)})};let it=null;function We(M){it&&it(M)}function rt(){Mt.stop()}function st(){Mt.start()}const Mt=new Kc;Mt.setAnimationLoop(We),typeof self<"u"&&Mt.setContext(self),this.setAnimationLoop=function(M){it=M,_.setAnimationLoop(M),M===null?Mt.stop():Mt.start()},_.addEventListener("sessionstart",rt),_.addEventListener("sessionend",st),this.render=function(M,F){if(F!==void 0&&F.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(A===!0)return;if(M.matrixWorldAutoUpdate===!0&&M.updateMatrixWorld(),F.parent===null&&F.matrixWorldAutoUpdate===!0&&F.updateMatrixWorld(),_.enabled===!0&&_.isPresenting===!0&&(_.cameraAutoUpdate===!0&&_.updateCamera(F),F=_.getCamera()),M.isScene===!0&&M.onBeforeRender(S,M,F,P),m=de.get(M,w.length),m.init(F),w.push(m),j.multiplyMatrices(F.projectionMatrix,F.matrixWorldInverse),K.setFromProjectionMatrix(j),B=this.localClippingEnabled,D=he.init(this.clippingPlanes,B),x=be.get(M,p.length),x.init(),p.push(x),_.enabled===!0&&_.isPresenting===!0){const ue=S.xr.getDepthSensingMesh();ue!==null&&bt(ue,F,-1/0,S.sortObjects)}bt(M,F,0,S.sortObjects),x.finish(),S.sortObjects===!0&&x.sort(v,T),ve=_.enabled===!1||_.isPresenting===!1||_.hasDepthSensing()===!1,ve&&ce.addToRenderList(x,M),this.info.render.frame++,D===!0&&he.beginShadows();const J=m.state.shadowsArray;Ne.render(J,M,F),D===!0&&he.endShadows(),this.info.autoReset===!0&&this.info.reset();const Q=x.opaque,W=x.transmissive;if(m.setupLights(),F.isArrayCamera){const ue=F.cameras;if(W.length>0)for(let me=0,_e=ue.length;me<_e;me++){const Se=ue[me];vn(Q,W,M,Se)}ve&&ce.render(M);for(let me=0,_e=ue.length;me<_e;me++){const Se=ue[me];Qt(x,M,Se,Se.viewport)}}else W.length>0&&vn(Q,W,M,F),ve&&ce.render(M),Qt(x,M,F);P!==null&&(Ae.updateMultisampleRenderTarget(P),Ae.updateRenderTargetMipmap(P)),M.isScene===!0&&M.onAfterRender(S,M,F),Oe.resetDefaultState(),Y=-1,E=null,w.pop(),w.length>0?(m=w[w.length-1],D===!0&&he.setGlobalState(S.clippingPlanes,m.state.camera)):m=null,p.pop(),p.length>0?x=p[p.length-1]:x=null};function bt(M,F,J,Q){if(M.visible===!1)return;if(M.layers.test(F.layers)){if(M.isGroup)J=M.renderOrder;else if(M.isLOD)M.autoUpdate===!0&&M.update(F);else if(M.isLight)m.pushLight(M),M.castShadow&&m.pushShadow(M);else if(M.isSprite){if(!M.frustumCulled||K.intersectsSprite(M)){Q&&X.setFromMatrixPosition(M.matrixWorld).applyMatrix4(j);const me=re.update(M),_e=M.material;_e.visible&&x.push(M,me,_e,J,X.z,null)}}else if((M.isMesh||M.isLine||M.isPoints)&&(!M.frustumCulled||K.intersectsObject(M))){const me=re.update(M),_e=M.material;if(Q&&(M.boundingSphere!==void 0?(M.boundingSphere===null&&M.computeBoundingSphere(),X.copy(M.boundingSphere.center)):(me.boundingSphere===null&&me.computeBoundingSphere(),X.copy(me.boundingSphere.center)),X.applyMatrix4(M.matrixWorld).applyMatrix4(j)),Array.isArray(_e)){const Se=me.groups;for(let Pe=0,Le=Se.length;Pe<Le;Pe++){const Ce=Se[Pe],qe=_e[Ce.materialIndex];qe&&qe.visible&&x.push(M,me,qe,J,X.z,Ce)}}else _e.visible&&x.push(M,me,_e,J,X.z,null)}}const ue=M.children;for(let me=0,_e=ue.length;me<_e;me++)bt(ue[me],F,J,Q)}function Qt(M,F,J,Q){const W=M.opaque,ue=M.transmissive,me=M.transparent;m.setupLightsView(J),D===!0&&he.setGlobalState(S.clippingPlanes,J),Q&&Me.viewport(b.copy(Q)),W.length>0&&xn(W,F,J),ue.length>0&&xn(ue,F,J),me.length>0&&xn(me,F,J),Me.buffers.depth.setTest(!0),Me.buffers.depth.setMask(!0),Me.buffers.color.setMask(!0),Me.setPolygonOffset(!1)}function vn(M,F,J,Q){if((J.isScene===!0?J.overrideMaterial:null)!==null)return;m.state.transmissionRenderTarget[Q.id]===void 0&&(m.state.transmissionRenderTarget[Q.id]=new Un(1,1,{generateMipmaps:!0,type:we.has("EXT_color_buffer_half_float")||we.has("EXT_color_buffer_float")?br:mn,minFilter:Ln,samples:4,stencilBuffer:s,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:je.workingColorSpace}));const ue=m.state.transmissionRenderTarget[Q.id],me=Q.viewport||b;ue.setSize(me.z,me.w);const _e=S.getRenderTarget();S.setRenderTarget(ue),S.getClearColor(O),$=S.getClearAlpha(),$<1&&S.setClearColor(16777215,.5),ve?ce.render(J):S.clear();const Se=S.toneMapping;S.toneMapping=fn;const Pe=Q.viewport;if(Q.viewport!==void 0&&(Q.viewport=void 0),m.setupLightsView(Q),D===!0&&he.setGlobalState(S.clippingPlanes,Q),xn(M,J,Q),Ae.updateMultisampleRenderTarget(ue),Ae.updateRenderTargetMipmap(ue),we.has("WEBGL_multisampled_render_to_texture")===!1){let Le=!1;for(let Ce=0,qe=F.length;Ce<qe;Ce++){const tt=F[Ce],nt=tt.object,Et=tt.geometry,Ye=tt.material,Te=tt.group;if(Ye.side===Zt&&nt.layers.test(Q.layers)){const gt=Ye.side;Ye.side=yt,Ye.needsUpdate=!0,Ea(nt,J,Q,Et,Ye,Te),Ye.side=gt,Ye.needsUpdate=!0,Le=!0}}Le===!0&&(Ae.updateMultisampleRenderTarget(ue),Ae.updateRenderTargetMipmap(ue))}S.setRenderTarget(_e),S.setClearColor(O,$),Pe!==void 0&&(Q.viewport=Pe),S.toneMapping=Se}function xn(M,F,J){const Q=F.isScene===!0?F.overrideMaterial:null;for(let W=0,ue=M.length;W<ue;W++){const me=M[W],_e=me.object,Se=me.geometry,Pe=Q===null?me.material:Q,Le=me.group;_e.layers.test(J.layers)&&Ea(_e,F,J,Se,Pe,Le)}}function Ea(M,F,J,Q,W,ue){M.onBeforeRender(S,F,J,Q,W,ue),M.modelViewMatrix.multiplyMatrices(J.matrixWorldInverse,M.matrixWorld),M.normalMatrix.getNormalMatrix(M.modelViewMatrix),W.onBeforeRender(S,F,J,Q,M,ue),W.transparent===!0&&W.side===Zt&&W.forceSinglePass===!1?(W.side=yt,W.needsUpdate=!0,S.renderBufferDirect(J,F,Q,W,M,ue),W.side=pn,W.needsUpdate=!0,S.renderBufferDirect(J,F,Q,W,M,ue),W.side=Zt):S.renderBufferDirect(J,F,Q,W,M,ue),M.onAfterRender(S,F,J,Q,W,ue)}function Di(M,F,J){F.isScene!==!0&&(F=le);const Q=Ie.get(M),W=m.state.lights,ue=m.state.shadowsArray,me=W.state.version,_e=ae.getParameters(M,W.state,ue,F,J),Se=ae.getProgramCacheKey(_e);let Pe=Q.programs;Q.environment=M.isMeshStandardMaterial?F.environment:null,Q.fog=F.fog,Q.envMap=(M.isMeshStandardMaterial?R:Ve).get(M.envMap||Q.environment),Q.envMapRotation=Q.environment!==null&&M.envMap===null?F.environmentRotation:M.envMapRotation,Pe===void 0&&(M.addEventListener("dispose",se),Pe=new Map,Q.programs=Pe);let Le=Pe.get(Se);if(Le!==void 0){if(Q.currentProgram===Le&&Q.lightsStateVersion===me)return Ta(M,_e),Le}else _e.uniforms=ae.getUniforms(M),M.onBuild(J,_e,S),M.onBeforeCompile(_e,S),Le=ae.acquireProgram(_e,Se),Pe.set(Se,Le),Q.uniforms=_e.uniforms;const Ce=Q.uniforms;return(!M.isShaderMaterial&&!M.isRawShaderMaterial||M.clipping===!0)&&(Ce.clippingPlanes=he.uniform),Ta(M,_e),Q.needsLights=hu(M),Q.lightsStateVersion=me,Q.needsLights&&(Ce.ambientLightColor.value=W.state.ambient,Ce.lightProbe.value=W.state.probe,Ce.directionalLights.value=W.state.directional,Ce.directionalLightShadows.value=W.state.directionalShadow,Ce.spotLights.value=W.state.spot,Ce.spotLightShadows.value=W.state.spotShadow,Ce.rectAreaLights.value=W.state.rectArea,Ce.ltc_1.value=W.state.rectAreaLTC1,Ce.ltc_2.value=W.state.rectAreaLTC2,Ce.pointLights.value=W.state.point,Ce.pointLightShadows.value=W.state.pointShadow,Ce.hemisphereLights.value=W.state.hemi,Ce.directionalShadowMap.value=W.state.directionalShadowMap,Ce.directionalShadowMatrix.value=W.state.directionalShadowMatrix,Ce.spotShadowMap.value=W.state.spotShadowMap,Ce.spotLightMatrix.value=W.state.spotLightMatrix,Ce.spotLightMap.value=W.state.spotLightMap,Ce.pointShadowMap.value=W.state.pointShadowMap,Ce.pointShadowMatrix.value=W.state.pointShadowMatrix),Q.currentProgram=Le,Q.uniformsList=null,Le}function wa(M){if(M.uniformsList===null){const F=M.currentProgram.getUniforms();M.uniformsList=hr.seqWithValue(F.seq,M.uniforms)}return M.uniformsList}function Ta(M,F){const J=Ie.get(M);J.outputColorSpace=F.outputColorSpace,J.batching=F.batching,J.batchingColor=F.batchingColor,J.instancing=F.instancing,J.instancingColor=F.instancingColor,J.instancingMorph=F.instancingMorph,J.skinning=F.skinning,J.morphTargets=F.morphTargets,J.morphNormals=F.morphNormals,J.morphColors=F.morphColors,J.morphTargetsCount=F.morphTargetsCount,J.numClippingPlanes=F.numClippingPlanes,J.numIntersection=F.numClipIntersection,J.vertexAlphas=F.vertexAlphas,J.vertexTangents=F.vertexTangents,J.toneMapping=F.toneMapping}function cu(M,F,J,Q,W){F.isScene!==!0&&(F=le),Ae.resetTextureUnits();const ue=F.fog,me=Q.isMeshStandardMaterial?F.environment:null,_e=P===null?S.outputColorSpace:P.isXRRenderTarget===!0?P.texture.colorSpace:_n,Se=(Q.isMeshStandardMaterial?R:Ve).get(Q.envMap||me),Pe=Q.vertexColors===!0&&!!J.attributes.color&&J.attributes.color.itemSize===4,Le=!!J.attributes.tangent&&(!!Q.normalMap||Q.anisotropy>0),Ce=!!J.morphAttributes.position,qe=!!J.morphAttributes.normal,tt=!!J.morphAttributes.color;let nt=fn;Q.toneMapped&&(P===null||P.isXRRenderTarget===!0)&&(nt=S.toneMapping);const Et=J.morphAttributes.position||J.morphAttributes.normal||J.morphAttributes.color,Ye=Et!==void 0?Et.length:0,Te=Ie.get(Q),gt=m.state.lights;if(D===!0&&(B===!0||M!==E)){const At=M===E&&Q.id===Y;he.setState(Q,M,At)}let $e=!1;Q.version===Te.__version?(Te.needsLights&&Te.lightsStateVersion!==gt.state.version||Te.outputColorSpace!==_e||W.isBatchedMesh&&Te.batching===!1||!W.isBatchedMesh&&Te.batching===!0||W.isBatchedMesh&&Te.batchingColor===!0&&W.colorTexture===null||W.isBatchedMesh&&Te.batchingColor===!1&&W.colorTexture!==null||W.isInstancedMesh&&Te.instancing===!1||!W.isInstancedMesh&&Te.instancing===!0||W.isSkinnedMesh&&Te.skinning===!1||!W.isSkinnedMesh&&Te.skinning===!0||W.isInstancedMesh&&Te.instancingColor===!0&&W.instanceColor===null||W.isInstancedMesh&&Te.instancingColor===!1&&W.instanceColor!==null||W.isInstancedMesh&&Te.instancingMorph===!0&&W.morphTexture===null||W.isInstancedMesh&&Te.instancingMorph===!1&&W.morphTexture!==null||Te.envMap!==Se||Q.fog===!0&&Te.fog!==ue||Te.numClippingPlanes!==void 0&&(Te.numClippingPlanes!==he.numPlanes||Te.numIntersection!==he.numIntersection)||Te.vertexAlphas!==Pe||Te.vertexTangents!==Le||Te.morphTargets!==Ce||Te.morphNormals!==qe||Te.morphColors!==tt||Te.toneMapping!==nt||Te.morphTargetsCount!==Ye)&&($e=!0):($e=!0,Te.__version=Q.version);let Wt=Te.currentProgram;$e===!0&&(Wt=Di(Q,F,W));let Ii=!1,yn=!1,Cr=!1;const ct=Wt.getUniforms(),en=Te.uniforms;if(Me.useProgram(Wt.program)&&(Ii=!0,yn=!0,Cr=!0),Q.id!==Y&&(Y=Q.id,yn=!0),Ii||E!==M){ct.setValue(L,"projectionMatrix",M.projectionMatrix),ct.setValue(L,"viewMatrix",M.matrixWorldInverse);const At=ct.map.cameraPosition;At!==void 0&&At.setValue(L,X.setFromMatrixPosition(M.matrixWorld)),Xe.logarithmicDepthBuffer&&ct.setValue(L,"logDepthBufFC",2/(Math.log(M.far+1)/Math.LN2)),(Q.isMeshPhongMaterial||Q.isMeshToonMaterial||Q.isMeshLambertMaterial||Q.isMeshBasicMaterial||Q.isMeshStandardMaterial||Q.isShaderMaterial)&&ct.setValue(L,"isOrthographic",M.isOrthographicCamera===!0),E!==M&&(E=M,yn=!0,Cr=!0)}if(W.isSkinnedMesh){ct.setOptional(L,W,"bindMatrix"),ct.setOptional(L,W,"bindMatrixInverse");const At=W.skeleton;At&&(At.boneTexture===null&&At.computeBoneTexture(),ct.setValue(L,"boneTexture",At.boneTexture,Ae))}W.isBatchedMesh&&(ct.setOptional(L,W,"batchingTexture"),ct.setValue(L,"batchingTexture",W._matricesTexture,Ae),ct.setOptional(L,W,"batchingColorTexture"),W._colorsTexture!==null&&ct.setValue(L,"batchingColorTexture",W._colorsTexture,Ae));const Rr=J.morphAttributes;if((Rr.position!==void 0||Rr.normal!==void 0||Rr.color!==void 0)&&ye.update(W,J,Wt),(yn||Te.receiveShadow!==W.receiveShadow)&&(Te.receiveShadow=W.receiveShadow,ct.setValue(L,"receiveShadow",W.receiveShadow)),Q.isMeshGouraudMaterial&&Q.envMap!==null&&(en.envMap.value=Se,en.flipEnvMap.value=Se.isCubeTexture&&Se.isRenderTargetTexture===!1?-1:1),Q.isMeshStandardMaterial&&Q.envMap===null&&F.environment!==null&&(en.envMapIntensity.value=F.environmentIntensity),yn&&(ct.setValue(L,"toneMappingExposure",S.toneMappingExposure),Te.needsLights&&uu(en,Cr),ue&&Q.fog===!0&&oe.refreshFogUniforms(en,ue),oe.refreshMaterialUniforms(en,Q,te,q,m.state.transmissionRenderTarget[M.id]),hr.upload(L,wa(Te),en,Ae)),Q.isShaderMaterial&&Q.uniformsNeedUpdate===!0&&(hr.upload(L,wa(Te),en,Ae),Q.uniformsNeedUpdate=!1),Q.isSpriteMaterial&&ct.setValue(L,"center",W.center),ct.setValue(L,"modelViewMatrix",W.modelViewMatrix),ct.setValue(L,"normalMatrix",W.normalMatrix),ct.setValue(L,"modelMatrix",W.matrixWorld),Q.isShaderMaterial||Q.isRawShaderMaterial){const At=Q.uniformsGroups;for(let Pr=0,du=At.length;Pr<du;Pr++){const Aa=At[Pr];Fe.update(Aa,Wt),Fe.bind(Aa,Wt)}}return Wt}function uu(M,F){M.ambientLightColor.needsUpdate=F,M.lightProbe.needsUpdate=F,M.directionalLights.needsUpdate=F,M.directionalLightShadows.needsUpdate=F,M.pointLights.needsUpdate=F,M.pointLightShadows.needsUpdate=F,M.spotLights.needsUpdate=F,M.spotLightShadows.needsUpdate=F,M.rectAreaLights.needsUpdate=F,M.hemisphereLights.needsUpdate=F}function hu(M){return M.isMeshLambertMaterial||M.isMeshToonMaterial||M.isMeshPhongMaterial||M.isMeshStandardMaterial||M.isShadowMaterial||M.isShaderMaterial&&M.lights===!0}this.getActiveCubeFace=function(){return k},this.getActiveMipmapLevel=function(){return I},this.getRenderTarget=function(){return P},this.setRenderTargetTextures=function(M,F,J){Ie.get(M.texture).__webglTexture=F,Ie.get(M.depthTexture).__webglTexture=J;const Q=Ie.get(M);Q.__hasExternalTextures=!0,Q.__autoAllocateDepthBuffer=J===void 0,Q.__autoAllocateDepthBuffer||we.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),Q.__useRenderToTexture=!1)},this.setRenderTargetFramebuffer=function(M,F){const J=Ie.get(M);J.__webglFramebuffer=F,J.__useDefaultFramebuffer=F===void 0},this.setRenderTarget=function(M,F=0,J=0){P=M,k=F,I=J;let Q=!0,W=null,ue=!1,me=!1;if(M){const Se=Ie.get(M);Se.__useDefaultFramebuffer!==void 0?(Me.bindFramebuffer(L.FRAMEBUFFER,null),Q=!1):Se.__webglFramebuffer===void 0?Ae.setupRenderTarget(M):Se.__hasExternalTextures&&Ae.rebindTextures(M,Ie.get(M.texture).__webglTexture,Ie.get(M.depthTexture).__webglTexture);const Pe=M.texture;(Pe.isData3DTexture||Pe.isDataArrayTexture||Pe.isCompressedArrayTexture)&&(me=!0);const Le=Ie.get(M).__webglFramebuffer;M.isWebGLCubeRenderTarget?(Array.isArray(Le[F])?W=Le[F][J]:W=Le[F],ue=!0):M.samples>0&&Ae.useMultisampledRTT(M)===!1?W=Ie.get(M).__webglMultisampledFramebuffer:Array.isArray(Le)?W=Le[J]:W=Le,b.copy(M.viewport),C.copy(M.scissor),H=M.scissorTest}else b.copy(U).multiplyScalar(te).floor(),C.copy(N).multiplyScalar(te).floor(),H=V;if(Me.bindFramebuffer(L.FRAMEBUFFER,W)&&Q&&Me.drawBuffers(M,W),Me.viewport(b),Me.scissor(C),Me.setScissorTest(H),ue){const Se=Ie.get(M.texture);L.framebufferTexture2D(L.FRAMEBUFFER,L.COLOR_ATTACHMENT0,L.TEXTURE_CUBE_MAP_POSITIVE_X+F,Se.__webglTexture,J)}else if(me){const Se=Ie.get(M.texture),Pe=F||0;L.framebufferTextureLayer(L.FRAMEBUFFER,L.COLOR_ATTACHMENT0,Se.__webglTexture,J||0,Pe)}Y=-1},this.readRenderTargetPixels=function(M,F,J,Q,W,ue,me){if(!(M&&M.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let _e=Ie.get(M).__webglFramebuffer;if(M.isWebGLCubeRenderTarget&&me!==void 0&&(_e=_e[me]),_e){Me.bindFramebuffer(L.FRAMEBUFFER,_e);try{const Se=M.texture,Pe=Se.format,Le=Se.type;if(!Xe.textureFormatReadable(Pe)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!Xe.textureTypeReadable(Le)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}F>=0&&F<=M.width-Q&&J>=0&&J<=M.height-W&&L.readPixels(F,J,Q,W,pe.convert(Pe),pe.convert(Le),ue)}finally{const Se=P!==null?Ie.get(P).__webglFramebuffer:null;Me.bindFramebuffer(L.FRAMEBUFFER,Se)}}},this.readRenderTargetPixelsAsync=async function(M,F,J,Q,W,ue,me){if(!(M&&M.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let _e=Ie.get(M).__webglFramebuffer;if(M.isWebGLCubeRenderTarget&&me!==void 0&&(_e=_e[me]),_e){Me.bindFramebuffer(L.FRAMEBUFFER,_e);try{const Se=M.texture,Pe=Se.format,Le=Se.type;if(!Xe.textureFormatReadable(Pe))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!Xe.textureTypeReadable(Le))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");if(F>=0&&F<=M.width-Q&&J>=0&&J<=M.height-W){const Ce=L.createBuffer();L.bindBuffer(L.PIXEL_PACK_BUFFER,Ce),L.bufferData(L.PIXEL_PACK_BUFFER,ue.byteLength,L.STREAM_READ),L.readPixels(F,J,Q,W,pe.convert(Pe),pe.convert(Le),0),L.flush();const qe=L.fenceSync(L.SYNC_GPU_COMMANDS_COMPLETE,0);await Gd(L,qe,4);try{L.bindBuffer(L.PIXEL_PACK_BUFFER,Ce),L.getBufferSubData(L.PIXEL_PACK_BUFFER,0,ue)}finally{L.deleteBuffer(Ce),L.deleteSync(qe)}return ue}}finally{const Se=P!==null?Ie.get(P).__webglFramebuffer:null;Me.bindFramebuffer(L.FRAMEBUFFER,Se)}}},this.copyFramebufferToTexture=function(M,F=null,J=0){M.isTexture!==!0&&(console.warn("WebGLRenderer: copyFramebufferToTexture function signature has changed."),F=arguments[0]||null,M=arguments[1]);const Q=Math.pow(2,-J),W=Math.floor(M.image.width*Q),ue=Math.floor(M.image.height*Q),me=F!==null?F.x:0,_e=F!==null?F.y:0;Ae.setTexture2D(M,0),L.copyTexSubImage2D(L.TEXTURE_2D,J,0,0,me,_e,W,ue),Me.unbindTexture()},this.copyTextureToTexture=function(M,F,J=null,Q=null,W=0){M.isTexture!==!0&&(console.warn("WebGLRenderer: copyTextureToTexture function signature has changed."),Q=arguments[0]||null,M=arguments[1],F=arguments[2],W=arguments[3]||0,J=null);let ue,me,_e,Se,Pe,Le;J!==null?(ue=J.max.x-J.min.x,me=J.max.y-J.min.y,_e=J.min.x,Se=J.min.y):(ue=M.image.width,me=M.image.height,_e=0,Se=0),Q!==null?(Pe=Q.x,Le=Q.y):(Pe=0,Le=0);const Ce=pe.convert(F.format),qe=pe.convert(F.type);Ae.setTexture2D(F,0),L.pixelStorei(L.UNPACK_FLIP_Y_WEBGL,F.flipY),L.pixelStorei(L.UNPACK_PREMULTIPLY_ALPHA_WEBGL,F.premultiplyAlpha),L.pixelStorei(L.UNPACK_ALIGNMENT,F.unpackAlignment);const tt=L.getParameter(L.UNPACK_ROW_LENGTH),nt=L.getParameter(L.UNPACK_IMAGE_HEIGHT),Et=L.getParameter(L.UNPACK_SKIP_PIXELS),Ye=L.getParameter(L.UNPACK_SKIP_ROWS),Te=L.getParameter(L.UNPACK_SKIP_IMAGES),gt=M.isCompressedTexture?M.mipmaps[W]:M.image;L.pixelStorei(L.UNPACK_ROW_LENGTH,gt.width),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,gt.height),L.pixelStorei(L.UNPACK_SKIP_PIXELS,_e),L.pixelStorei(L.UNPACK_SKIP_ROWS,Se),M.isDataTexture?L.texSubImage2D(L.TEXTURE_2D,W,Pe,Le,ue,me,Ce,qe,gt.data):M.isCompressedTexture?L.compressedTexSubImage2D(L.TEXTURE_2D,W,Pe,Le,gt.width,gt.height,Ce,gt.data):L.texSubImage2D(L.TEXTURE_2D,W,Pe,Le,Ce,qe,gt),L.pixelStorei(L.UNPACK_ROW_LENGTH,tt),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,nt),L.pixelStorei(L.UNPACK_SKIP_PIXELS,Et),L.pixelStorei(L.UNPACK_SKIP_ROWS,Ye),L.pixelStorei(L.UNPACK_SKIP_IMAGES,Te),W===0&&F.generateMipmaps&&L.generateMipmap(L.TEXTURE_2D),Me.unbindTexture()},this.copyTextureToTexture3D=function(M,F,J=null,Q=null,W=0){M.isTexture!==!0&&(console.warn("WebGLRenderer: copyTextureToTexture3D function signature has changed."),J=arguments[0]||null,Q=arguments[1]||null,M=arguments[2],F=arguments[3],W=arguments[4]||0);let ue,me,_e,Se,Pe,Le,Ce,qe,tt;const nt=M.isCompressedTexture?M.mipmaps[W]:M.image;J!==null?(ue=J.max.x-J.min.x,me=J.max.y-J.min.y,_e=J.max.z-J.min.z,Se=J.min.x,Pe=J.min.y,Le=J.min.z):(ue=nt.width,me=nt.height,_e=nt.depth,Se=0,Pe=0,Le=0),Q!==null?(Ce=Q.x,qe=Q.y,tt=Q.z):(Ce=0,qe=0,tt=0);const Et=pe.convert(F.format),Ye=pe.convert(F.type);let Te;if(F.isData3DTexture)Ae.setTexture3D(F,0),Te=L.TEXTURE_3D;else if(F.isDataArrayTexture||F.isCompressedArrayTexture)Ae.setTexture2DArray(F,0),Te=L.TEXTURE_2D_ARRAY;else{console.warn("THREE.WebGLRenderer.copyTextureToTexture3D: only supports THREE.DataTexture3D and THREE.DataTexture2DArray.");return}L.pixelStorei(L.UNPACK_FLIP_Y_WEBGL,F.flipY),L.pixelStorei(L.UNPACK_PREMULTIPLY_ALPHA_WEBGL,F.premultiplyAlpha),L.pixelStorei(L.UNPACK_ALIGNMENT,F.unpackAlignment);const gt=L.getParameter(L.UNPACK_ROW_LENGTH),$e=L.getParameter(L.UNPACK_IMAGE_HEIGHT),Wt=L.getParameter(L.UNPACK_SKIP_PIXELS),Ii=L.getParameter(L.UNPACK_SKIP_ROWS),yn=L.getParameter(L.UNPACK_SKIP_IMAGES);L.pixelStorei(L.UNPACK_ROW_LENGTH,nt.width),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,nt.height),L.pixelStorei(L.UNPACK_SKIP_PIXELS,Se),L.pixelStorei(L.UNPACK_SKIP_ROWS,Pe),L.pixelStorei(L.UNPACK_SKIP_IMAGES,Le),M.isDataTexture||M.isData3DTexture?L.texSubImage3D(Te,W,Ce,qe,tt,ue,me,_e,Et,Ye,nt.data):F.isCompressedArrayTexture?L.compressedTexSubImage3D(Te,W,Ce,qe,tt,ue,me,_e,Et,nt.data):L.texSubImage3D(Te,W,Ce,qe,tt,ue,me,_e,Et,Ye,nt),L.pixelStorei(L.UNPACK_ROW_LENGTH,gt),L.pixelStorei(L.UNPACK_IMAGE_HEIGHT,$e),L.pixelStorei(L.UNPACK_SKIP_PIXELS,Wt),L.pixelStorei(L.UNPACK_SKIP_ROWS,Ii),L.pixelStorei(L.UNPACK_SKIP_IMAGES,yn),W===0&&F.generateMipmaps&&L.generateMipmap(Te),Me.unbindTexture()},this.initRenderTarget=function(M){Ie.get(M).__webglFramebuffer===void 0&&Ae.setupRenderTarget(M)},this.initTexture=function(M){M.isCubeTexture?Ae.setTextureCube(M,0):M.isData3DTexture?Ae.setTexture3D(M,0):M.isDataArrayTexture||M.isCompressedArrayTexture?Ae.setTexture2DArray(M,0):Ae.setTexture2D(M,0),Me.unbindTexture()},this.resetState=function(){k=0,I=0,P=null,Me.reset(),Oe.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return Jt}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(e){this._outputColorSpace=e;const t=this.getContext();t.drawingBufferColorSpace=e===ya?"display-p3":"srgb",t.unpackColorSpace=je.workingColorSpace===Er?"display-p3":"srgb"}}class Z_ extends dt{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new Ht,this.environmentIntensity=1,this.environmentRotation=new Ht,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(e,t){return super.copy(e,t),e.background!==null&&(this.background=e.background.clone()),e.environment!==null&&(this.environment=e.environment.clone()),e.fog!==null&&(this.fog=e.fog.clone()),this.backgroundBlurriness=e.backgroundBlurriness,this.backgroundIntensity=e.backgroundIntensity,this.backgroundRotation.copy(e.backgroundRotation),this.environmentIntensity=e.environmentIntensity,this.environmentRotation.copy(e.environmentRotation),e.overrideMaterial!==null&&(this.overrideMaterial=e.overrideMaterial.clone()),this.matrixAutoUpdate=e.matrixAutoUpdate,this}toJSON(e){const t=super.toJSON(e);return this.fog!==null&&(t.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(t.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(t.object.backgroundIntensity=this.backgroundIntensity),t.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(t.object.environmentIntensity=this.environmentIntensity),t.object.environmentRotation=this.environmentRotation.toArray(),t}}class ba extends Bn{constructor(e){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new He(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.linewidth=e.linewidth,this.linecap=e.linecap,this.linejoin=e.linejoin,this.fog=e.fog,this}}const xr=new G,yr=new G,ql=new Je,bi=new wr,ir=new Ui,ta=new G,Yl=new G;class J_ extends dt{constructor(e=new Lt,t=new ba){super(),this.isLine=!0,this.type="Line",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,n=[0];for(let r=1,s=t.count;r<s;r++)xr.fromBufferAttribute(t,r-1),yr.fromBufferAttribute(t,r),n[r]=n[r-1],n[r]+=xr.distanceTo(yr);e.setAttribute("lineDistance",new ft(n,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(e,t){const n=this.geometry,r=this.matrixWorld,s=e.params.Line.threshold,o=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),ir.copy(n.boundingSphere),ir.applyMatrix4(r),ir.radius+=s,e.ray.intersectsSphere(ir)===!1)return;ql.copy(r).invert(),bi.copy(e.ray).applyMatrix4(ql);const a=s/((this.scale.x+this.scale.y+this.scale.z)/3),l=a*a,c=this.isLineSegments?2:1,u=n.index,d=n.attributes.position;if(u!==null){const f=Math.max(0,o.start),g=Math.min(u.count,o.start+o.count);for(let x=f,m=g-1;x<m;x+=c){const p=u.getX(x),w=u.getX(x+1),S=rr(this,e,bi,l,p,w);S&&t.push(S)}if(this.isLineLoop){const x=u.getX(g-1),m=u.getX(f),p=rr(this,e,bi,l,x,m);p&&t.push(p)}}else{const f=Math.max(0,o.start),g=Math.min(d.count,o.start+o.count);for(let x=f,m=g-1;x<m;x+=c){const p=rr(this,e,bi,l,x,x+1);p&&t.push(p)}if(this.isLineLoop){const x=rr(this,e,bi,l,g-1,f);x&&t.push(x)}}}updateMorphTargets(){const t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){const r=t[n[0]];if(r!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,o=r.length;s<o;s++){const a=r[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=s}}}}}function rr(i,e,t,n,r,s){const o=i.geometry.attributes.position;if(xr.fromBufferAttribute(o,r),yr.fromBufferAttribute(o,s),t.distanceSqToSegment(xr,yr,ta,Yl)>n)return;ta.applyMatrix4(i.matrixWorld);const l=e.ray.origin.distanceTo(ta);if(!(l<e.near||l>e.far))return{distance:l,point:Yl.clone().applyMatrix4(i.matrixWorld),index:r,face:null,faceIndex:null,object:i}}const jl=new G,$l=new G;class ru extends J_{constructor(e,t){super(e,t),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const e=this.geometry;if(e.index===null){const t=e.attributes.position,n=[];for(let r=0,s=t.count;r<s;r+=2)jl.fromBufferAttribute(t,r),$l.fromBufferAttribute(t,r+1),n[r]=r===0?0:n[r-1],n[r+1]=n[r]+jl.distanceTo($l);e.setAttribute("lineDistance",new ft(n,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class su extends Bn{constructor(e){super(),this.isPointsMaterial=!0,this.type="PointsMaterial",this.color=new He(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.color.copy(e.color),this.map=e.map,this.alphaMap=e.alphaMap,this.size=e.size,this.sizeAttenuation=e.sizeAttenuation,this.fog=e.fog,this}}const Kl=new Je,fa=new wr,sr=new Ui,ar=new G;class Q_ extends dt{constructor(e=new Lt,t=new su){super(),this.isPoints=!0,this.type="Points",this.geometry=e,this.material=t,this.updateMorphTargets()}copy(e,t){return super.copy(e,t),this.material=Array.isArray(e.material)?e.material.slice():e.material,this.geometry=e.geometry,this}raycast(e,t){const n=this.geometry,r=this.matrixWorld,s=e.params.Points.threshold,o=n.drawRange;if(n.boundingSphere===null&&n.computeBoundingSphere(),sr.copy(n.boundingSphere),sr.applyMatrix4(r),sr.radius+=s,e.ray.intersectsSphere(sr)===!1)return;Kl.copy(r).invert(),fa.copy(e.ray).applyMatrix4(Kl);const a=s/((this.scale.x+this.scale.y+this.scale.z)/3),l=a*a,c=n.index,h=n.attributes.position;if(c!==null){const d=Math.max(0,o.start),f=Math.min(c.count,o.start+o.count);for(let g=d,x=f;g<x;g++){const m=c.getX(g);ar.fromBufferAttribute(h,m),Zl(ar,m,l,r,e,t,this)}}else{const d=Math.max(0,o.start),f=Math.min(h.count,o.start+o.count);for(let g=d,x=f;g<x;g++)ar.fromBufferAttribute(h,g),Zl(ar,g,l,r,e,t,this)}}updateMorphTargets(){const t=this.geometry.morphAttributes,n=Object.keys(t);if(n.length>0){const r=t[n[0]];if(r!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let s=0,o=r.length;s<o;s++){const a=r[s].name||String(s);this.morphTargetInfluences.push(0),this.morphTargetDictionary[a]=s}}}}}function Zl(i,e,t,n,r,s,o){const a=fa.distanceSqToPoint(i);if(a<t){const l=new G;fa.closestPointToPoint(i,l),l.applyMatrix4(n);const c=r.ray.origin.distanceTo(l);if(c<r.near||c>r.far)return;s.push({distance:c,distanceToRay:Math.sqrt(a),point:l,index:e,face:null,object:o})}}class Jl extends Bn{constructor(e){super(),this.isMeshStandardMaterial=!0,this.defines={STANDARD:""},this.type="MeshStandardMaterial",this.color=new He(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new He(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Oc,this.normalScale=new De(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new Ht,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(e)}copy(e){return super.copy(e),this.defines={STANDARD:""},this.color.copy(e.color),this.roughness=e.roughness,this.metalness=e.metalness,this.map=e.map,this.lightMap=e.lightMap,this.lightMapIntensity=e.lightMapIntensity,this.aoMap=e.aoMap,this.aoMapIntensity=e.aoMapIntensity,this.emissive.copy(e.emissive),this.emissiveMap=e.emissiveMap,this.emissiveIntensity=e.emissiveIntensity,this.bumpMap=e.bumpMap,this.bumpScale=e.bumpScale,this.normalMap=e.normalMap,this.normalMapType=e.normalMapType,this.normalScale.copy(e.normalScale),this.displacementMap=e.displacementMap,this.displacementScale=e.displacementScale,this.displacementBias=e.displacementBias,this.roughnessMap=e.roughnessMap,this.metalnessMap=e.metalnessMap,this.alphaMap=e.alphaMap,this.envMap=e.envMap,this.envMapRotation.copy(e.envMapRotation),this.envMapIntensity=e.envMapIntensity,this.wireframe=e.wireframe,this.wireframeLinewidth=e.wireframeLinewidth,this.wireframeLinecap=e.wireframeLinecap,this.wireframeLinejoin=e.wireframeLinejoin,this.flatShading=e.flatShading,this.fog=e.fog,this}}class au extends dt{constructor(e,t=1){super(),this.isLight=!0,this.type="Light",this.color=new He(e),this.intensity=t}dispose(){}copy(e,t){return super.copy(e,t),this.color.copy(e.color),this.intensity=e.intensity,this}toJSON(e){const t=super.toJSON(e);return t.object.color=this.color.getHex(),t.object.intensity=this.intensity,this.groundColor!==void 0&&(t.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(t.object.distance=this.distance),this.angle!==void 0&&(t.object.angle=this.angle),this.decay!==void 0&&(t.object.decay=this.decay),this.penumbra!==void 0&&(t.object.penumbra=this.penumbra),this.shadow!==void 0&&(t.object.shadow=this.shadow.toJSON()),t}}const na=new Je,Ql=new G,ec=new G;class ev{constructor(e){this.camera=e,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new De(512,512),this.map=null,this.mapPass=null,this.matrix=new Je,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new Sa,this._frameExtents=new De(1,1),this._viewportCount=1,this._viewports=[new ht(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(e){const t=this.camera,n=this.matrix;Ql.setFromMatrixPosition(e.matrixWorld),t.position.copy(Ql),ec.setFromMatrixPosition(e.target.matrixWorld),t.lookAt(ec),t.updateMatrixWorld(),na.multiplyMatrices(t.projectionMatrix,t.matrixWorldInverse),this._frustum.setFromProjectionMatrix(na),n.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),n.multiply(na)}getViewport(e){return this._viewports[e]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(e){return this.camera=e.camera.clone(),this.bias=e.bias,this.radius=e.radius,this.mapSize.copy(e.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const e={};return this.bias!==0&&(e.bias=this.bias),this.normalBias!==0&&(e.normalBias=this.normalBias),this.radius!==1&&(e.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(e.mapSize=this.mapSize.toArray()),e.camera=this.camera.toJSON(!1).object,delete e.camera.matrix,e}}class tv extends ev{constructor(){super(new Zc(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class nv extends au{constructor(e,t){super(e,t),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(dt.DEFAULT_UP),this.updateMatrix(),this.target=new dt,this.shadow=new tv}dispose(){this.shadow.dispose()}copy(e){return super.copy(e),this.target=e.target.clone(),this.shadow=e.shadow.clone(),this}}class iv extends au{constructor(e,t){super(e,t),this.isAmbientLight=!0,this.type="AmbientLight"}}class tc{constructor(e=1,t=0,n=0){return this.radius=e,this.phi=t,this.theta=n,this}set(e,t,n){return this.radius=e,this.phi=t,this.theta=n,this}copy(e){return this.radius=e.radius,this.phi=e.phi,this.theta=e.theta,this}makeSafe(){return this.phi=Math.max(1e-6,Math.min(Math.PI-1e-6,this.phi)),this}setFromVector3(e){return this.setFromCartesianCoords(e.x,e.y,e.z)}setFromCartesianCoords(e,t,n){return this.radius=Math.sqrt(e*e+t*t+n*n),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(e,n),this.phi=Math.acos(vt(t/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}class rv extends ru{constructor(e=10,t=10,n=4473924,r=8947848){n=new He(n),r=new He(r);const s=t/2,o=e/t,a=e/2,l=[],c=[];for(let d=0,f=0,g=-a;d<=t;d++,g+=o){l.push(-a,0,g,a,0,g),l.push(g,0,-a,g,0,a);const x=d===s?n:r;x.toArray(c,f),f+=3,x.toArray(c,f),f+=3,x.toArray(c,f),f+=3,x.toArray(c,f),f+=3}const u=new Lt;u.setAttribute("position",new ft(l,3)),u.setAttribute("color",new ft(c,3));const h=new ba({vertexColors:!0,toneMapped:!1});super(u,h),this.type="GridHelper"}dispose(){this.geometry.dispose(),this.material.dispose()}}class ia extends ru{constructor(e=1){const t=[0,0,0,e,0,0,0,0,0,0,e,0,0,0,0,0,0,e],n=[1,0,0,1,.6,0,0,1,0,.6,1,0,0,0,1,0,.6,1],r=new Lt;r.setAttribute("position",new ft(t,3)),r.setAttribute("color",new ft(n,3));const s=new ba({vertexColors:!0,toneMapped:!1});super(r,s),this.type="AxesHelper"}setColors(e,t,n){const r=new He,s=this.geometry.attributes.color.array;return r.set(e),r.toArray(s,0),r.toArray(s,3),r.set(t),r.toArray(s,6),r.toArray(s,9),r.set(n),r.toArray(s,12),r.toArray(s,15),this.geometry.attributes.color.needsUpdate=!0,this}dispose(){this.geometry.dispose(),this.material.dispose()}}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:xa}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=xa);const nc={type:"change"},ra={type:"start"},ic={type:"end"},or=new wr,rc=new ln,sv=Math.cos(70*zd.DEG2RAD);class av extends Fn{constructor(e,t){super(),this.object=e,this.domElement=t,this.domElement.style.touchAction="none",this.enabled=!0,this.target=new G,this.cursor=new G,this.minDistance=0,this.maxDistance=1/0,this.minZoom=0,this.maxZoom=1/0,this.minTargetRadius=0,this.maxTargetRadius=1/0,this.minPolarAngle=0,this.maxPolarAngle=Math.PI,this.minAzimuthAngle=-1/0,this.maxAzimuthAngle=1/0,this.enableDamping=!1,this.dampingFactor=.05,this.enableZoom=!0,this.zoomSpeed=1,this.enableRotate=!0,this.rotateSpeed=1,this.enablePan=!0,this.panSpeed=1,this.screenSpacePanning=!0,this.keyPanSpeed=7,this.zoomToCursor=!1,this.autoRotate=!1,this.autoRotateSpeed=2,this.keys={LEFT:"ArrowLeft",UP:"ArrowUp",RIGHT:"ArrowRight",BOTTOM:"ArrowDown"},this.mouseButtons={LEFT:kn.ROTATE,MIDDLE:kn.DOLLY,RIGHT:kn.PAN},this.touches={ONE:zn.ROTATE,TWO:zn.DOLLY_PAN},this.target0=this.target.clone(),this.position0=this.object.position.clone(),this.zoom0=this.object.zoom,this._domElementKeyEvents=null,this.getPolarAngle=function(){return a.phi},this.getAzimuthalAngle=function(){return a.theta},this.getDistance=function(){return this.object.position.distanceTo(this.target)},this.listenToKeyEvents=function(_){_.addEventListener("keydown",he),this._domElementKeyEvents=_},this.stopListenToKeyEvents=function(){this._domElementKeyEvents.removeEventListener("keydown",he),this._domElementKeyEvents=null},this.saveState=function(){n.target0.copy(n.target),n.position0.copy(n.object.position),n.zoom0=n.object.zoom},this.reset=function(){n.target.copy(n.target0),n.object.position.copy(n.position0),n.object.zoom=n.zoom0,n.object.updateProjectionMatrix(),n.dispatchEvent(nc),n.update(),s=r.NONE},this.update=function(){const _=new G,ne=new Gt().setFromUnitVectors(e.up,new G(0,1,0)),Z=ne.clone().invert(),ie=new G,se=new Gt,Ee=new G,Ue=2*Math.PI;return function(it=null){const We=n.object.position;_.copy(We).sub(n.target),_.applyQuaternion(ne),a.setFromVector3(_),n.autoRotate&&s===r.NONE&&H(b(it)),n.enableDamping?(a.theta+=l.theta*n.dampingFactor,a.phi+=l.phi*n.dampingFactor):(a.theta+=l.theta,a.phi+=l.phi);let rt=n.minAzimuthAngle,st=n.maxAzimuthAngle;isFinite(rt)&&isFinite(st)&&(rt<-Math.PI?rt+=Ue:rt>Math.PI&&(rt-=Ue),st<-Math.PI?st+=Ue:st>Math.PI&&(st-=Ue),rt<=st?a.theta=Math.max(rt,Math.min(st,a.theta)):a.theta=a.theta>(rt+st)/2?Math.max(rt,a.theta):Math.min(st,a.theta)),a.phi=Math.max(n.minPolarAngle,Math.min(n.maxPolarAngle,a.phi)),a.makeSafe(),n.enableDamping===!0?n.target.addScaledVector(u,n.dampingFactor):n.target.add(u),n.target.sub(n.cursor),n.target.clampLength(n.minTargetRadius,n.maxTargetRadius),n.target.add(n.cursor);let Mt=!1;if(n.zoomToCursor&&I||n.object.isOrthographicCamera)a.radius=U(a.radius);else{const bt=a.radius;a.radius=U(a.radius*c),Mt=bt!=a.radius}if(_.setFromSpherical(a),_.applyQuaternion(Z),We.copy(n.target).add(_),n.object.lookAt(n.target),n.enableDamping===!0?(l.theta*=1-n.dampingFactor,l.phi*=1-n.dampingFactor,u.multiplyScalar(1-n.dampingFactor)):(l.set(0,0,0),u.set(0,0,0)),n.zoomToCursor&&I){let bt=null;if(n.object.isPerspectiveCamera){const Qt=_.length();bt=U(Qt*c);const vn=Qt-bt;n.object.position.addScaledVector(A,vn),n.object.updateMatrixWorld(),Mt=!!vn}else if(n.object.isOrthographicCamera){const Qt=new G(k.x,k.y,0);Qt.unproject(n.object);const vn=n.object.zoom;n.object.zoom=Math.max(n.minZoom,Math.min(n.maxZoom,n.object.zoom/c)),n.object.updateProjectionMatrix(),Mt=vn!==n.object.zoom;const xn=new G(k.x,k.y,0);xn.unproject(n.object),n.object.position.sub(xn).add(Qt),n.object.updateMatrixWorld(),bt=_.length()}else console.warn("WARNING: OrbitControls.js encountered an unknown camera type - zoom to cursor disabled."),n.zoomToCursor=!1;bt!==null&&(this.screenSpacePanning?n.target.set(0,0,-1).transformDirection(n.object.matrix).multiplyScalar(bt).add(n.object.position):(or.origin.copy(n.object.position),or.direction.set(0,0,-1).transformDirection(n.object.matrix),Math.abs(n.object.up.dot(or.direction))<sv?e.lookAt(n.target):(rc.setFromNormalAndCoplanarPoint(n.object.up,n.target),or.intersectPlane(rc,n.target))))}else if(n.object.isOrthographicCamera){const bt=n.object.zoom;n.object.zoom=Math.max(n.minZoom,Math.min(n.maxZoom,n.object.zoom/c)),bt!==n.object.zoom&&(n.object.updateProjectionMatrix(),Mt=!0)}return c=1,I=!1,Mt||ie.distanceToSquared(n.object.position)>o||8*(1-se.dot(n.object.quaternion))>o||Ee.distanceToSquared(n.target)>o?(n.dispatchEvent(nc),ie.copy(n.object.position),se.copy(n.object.quaternion),Ee.copy(n.target),!0):!1}}(),this.dispose=function(){n.domElement.removeEventListener("contextmenu",ye),n.domElement.removeEventListener("pointerdown",Ve),n.domElement.removeEventListener("pointercancel",y),n.domElement.removeEventListener("wheel",ae),n.domElement.removeEventListener("pointermove",R),n.domElement.removeEventListener("pointerup",y),n.domElement.getRootNode().removeEventListener("keydown",be,{capture:!0}),n._domElementKeyEvents!==null&&(n._domElementKeyEvents.removeEventListener("keydown",he),n._domElementKeyEvents=null)};const n=this,r={NONE:-1,ROTATE:0,DOLLY:1,PAN:2,TOUCH_ROTATE:3,TOUCH_PAN:4,TOUCH_DOLLY_PAN:5,TOUCH_DOLLY_ROTATE:6};let s=r.NONE;const o=1e-6,a=new tc,l=new tc;let c=1;const u=new G,h=new De,d=new De,f=new De,g=new De,x=new De,m=new De,p=new De,w=new De,S=new De,A=new G,k=new De;let I=!1;const P=[],Y={};let E=!1;function b(_){return _!==null?2*Math.PI/60*n.autoRotateSpeed*_:2*Math.PI/60/60*n.autoRotateSpeed}function C(_){const ne=Math.abs(_*.01);return Math.pow(.95,n.zoomSpeed*ne)}function H(_){l.theta-=_}function O(_){l.phi-=_}const $=function(){const _=new G;return function(Z,ie){_.setFromMatrixColumn(ie,0),_.multiplyScalar(-Z),u.add(_)}}(),z=function(){const _=new G;return function(Z,ie){n.screenSpacePanning===!0?_.setFromMatrixColumn(ie,1):(_.setFromMatrixColumn(ie,0),_.crossVectors(n.object.up,_)),_.multiplyScalar(Z),u.add(_)}}(),q=function(){const _=new G;return function(Z,ie){const se=n.domElement;if(n.object.isPerspectiveCamera){const Ee=n.object.position;_.copy(Ee).sub(n.target);let Ue=_.length();Ue*=Math.tan(n.object.fov/2*Math.PI/180),$(2*Z*Ue/se.clientHeight,n.object.matrix),z(2*ie*Ue/se.clientHeight,n.object.matrix)}else n.object.isOrthographicCamera?($(Z*(n.object.right-n.object.left)/n.object.zoom/se.clientWidth,n.object.matrix),z(ie*(n.object.top-n.object.bottom)/n.object.zoom/se.clientHeight,n.object.matrix)):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - pan disabled."),n.enablePan=!1)}}();function te(_){n.object.isPerspectiveCamera||n.object.isOrthographicCamera?c/=_:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),n.enableZoom=!1)}function v(_){n.object.isPerspectiveCamera||n.object.isOrthographicCamera?c*=_:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),n.enableZoom=!1)}function T(_,ne){if(!n.zoomToCursor)return;I=!0;const Z=n.domElement.getBoundingClientRect(),ie=_-Z.left,se=ne-Z.top,Ee=Z.width,Ue=Z.height;k.x=ie/Ee*2-1,k.y=-(se/Ue)*2+1,A.set(k.x,k.y,1).unproject(n.object).sub(n.object.position).normalize()}function U(_){return Math.max(n.minDistance,Math.min(n.maxDistance,_))}function N(_){h.set(_.clientX,_.clientY)}function V(_){T(_.clientX,_.clientX),p.set(_.clientX,_.clientY)}function K(_){g.set(_.clientX,_.clientY)}function D(_){d.set(_.clientX,_.clientY),f.subVectors(d,h).multiplyScalar(n.rotateSpeed);const ne=n.domElement;H(2*Math.PI*f.x/ne.clientHeight),O(2*Math.PI*f.y/ne.clientHeight),h.copy(d),n.update()}function B(_){w.set(_.clientX,_.clientY),S.subVectors(w,p),S.y>0?te(C(S.y)):S.y<0&&v(C(S.y)),p.copy(w),n.update()}function j(_){x.set(_.clientX,_.clientY),m.subVectors(x,g).multiplyScalar(n.panSpeed),q(m.x,m.y),g.copy(x),n.update()}function X(_){T(_.clientX,_.clientY),_.deltaY<0?v(C(_.deltaY)):_.deltaY>0&&te(C(_.deltaY)),n.update()}function le(_){let ne=!1;switch(_.code){case n.keys.UP:_.ctrlKey||_.metaKey||_.shiftKey?O(2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):q(0,n.keyPanSpeed),ne=!0;break;case n.keys.BOTTOM:_.ctrlKey||_.metaKey||_.shiftKey?O(-2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):q(0,-n.keyPanSpeed),ne=!0;break;case n.keys.LEFT:_.ctrlKey||_.metaKey||_.shiftKey?H(2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):q(n.keyPanSpeed,0),ne=!0;break;case n.keys.RIGHT:_.ctrlKey||_.metaKey||_.shiftKey?H(-2*Math.PI*n.rotateSpeed/n.domElement.clientHeight):q(-n.keyPanSpeed,0),ne=!0;break}ne&&(_.preventDefault(),n.update())}function ve(_){if(P.length===1)h.set(_.pageX,_.pageY);else{const ne=Fe(_),Z=.5*(_.pageX+ne.x),ie=.5*(_.pageY+ne.y);h.set(Z,ie)}}function ge(_){if(P.length===1)g.set(_.pageX,_.pageY);else{const ne=Fe(_),Z=.5*(_.pageX+ne.x),ie=.5*(_.pageY+ne.y);g.set(Z,ie)}}function L(_){const ne=Fe(_),Z=_.pageX-ne.x,ie=_.pageY-ne.y,se=Math.sqrt(Z*Z+ie*ie);p.set(0,se)}function xe(_){n.enableZoom&&L(_),n.enablePan&&ge(_)}function we(_){n.enableZoom&&L(_),n.enableRotate&&ve(_)}function Xe(_){if(P.length==1)d.set(_.pageX,_.pageY);else{const Z=Fe(_),ie=.5*(_.pageX+Z.x),se=.5*(_.pageY+Z.y);d.set(ie,se)}f.subVectors(d,h).multiplyScalar(n.rotateSpeed);const ne=n.domElement;H(2*Math.PI*f.x/ne.clientHeight),O(2*Math.PI*f.y/ne.clientHeight),h.copy(d)}function Me(_){if(P.length===1)x.set(_.pageX,_.pageY);else{const ne=Fe(_),Z=.5*(_.pageX+ne.x),ie=.5*(_.pageY+ne.y);x.set(Z,ie)}m.subVectors(x,g).multiplyScalar(n.panSpeed),q(m.x,m.y),g.copy(x)}function Be(_){const ne=Fe(_),Z=_.pageX-ne.x,ie=_.pageY-ne.y,se=Math.sqrt(Z*Z+ie*ie);w.set(0,se),S.set(0,Math.pow(w.y/p.y,n.zoomSpeed)),te(S.y),p.copy(w);const Ee=(_.pageX+ne.x)*.5,Ue=(_.pageY+ne.y)*.5;T(Ee,Ue)}function Ie(_){n.enableZoom&&Be(_),n.enablePan&&Me(_)}function Ae(_){n.enableZoom&&Be(_),n.enableRotate&&Xe(_)}function Ve(_){n.enabled!==!1&&(P.length===0&&(n.domElement.setPointerCapture(_.pointerId),n.domElement.addEventListener("pointermove",R),n.domElement.addEventListener("pointerup",y)),!pe(_)&&(Ge(_),_.pointerType==="touch"?Ne(_):ee(_)))}function R(_){n.enabled!==!1&&(_.pointerType==="touch"?ce(_):re(_))}function y(_){switch(Re(_),P.length){case 0:n.domElement.releasePointerCapture(_.pointerId),n.domElement.removeEventListener("pointermove",R),n.domElement.removeEventListener("pointerup",y),n.dispatchEvent(ic),s=r.NONE;break;case 1:const ne=P[0],Z=Y[ne];Ne({pointerId:ne,pageX:Z.x,pageY:Z.y});break}}function ee(_){let ne;switch(_.button){case 0:ne=n.mouseButtons.LEFT;break;case 1:ne=n.mouseButtons.MIDDLE;break;case 2:ne=n.mouseButtons.RIGHT;break;default:ne=-1}switch(ne){case kn.DOLLY:if(n.enableZoom===!1)return;V(_),s=r.DOLLY;break;case kn.ROTATE:if(_.ctrlKey||_.metaKey||_.shiftKey){if(n.enablePan===!1)return;K(_),s=r.PAN}else{if(n.enableRotate===!1)return;N(_),s=r.ROTATE}break;case kn.PAN:if(_.ctrlKey||_.metaKey||_.shiftKey){if(n.enableRotate===!1)return;N(_),s=r.ROTATE}else{if(n.enablePan===!1)return;K(_),s=r.PAN}break;default:s=r.NONE}s!==r.NONE&&n.dispatchEvent(ra)}function re(_){switch(s){case r.ROTATE:if(n.enableRotate===!1)return;D(_);break;case r.DOLLY:if(n.enableZoom===!1)return;B(_);break;case r.PAN:if(n.enablePan===!1)return;j(_);break}}function ae(_){n.enabled===!1||n.enableZoom===!1||s!==r.NONE||(_.preventDefault(),n.dispatchEvent(ra),X(oe(_)),n.dispatchEvent(ic))}function oe(_){const ne=_.deltaMode,Z={clientX:_.clientX,clientY:_.clientY,deltaY:_.deltaY};switch(ne){case 1:Z.deltaY*=16;break;case 2:Z.deltaY*=100;break}return _.ctrlKey&&!E&&(Z.deltaY*=10),Z}function be(_){_.key==="Control"&&(E=!0,n.domElement.getRootNode().addEventListener("keyup",de,{passive:!0,capture:!0}))}function de(_){_.key==="Control"&&(E=!1,n.domElement.getRootNode().removeEventListener("keyup",de,{passive:!0,capture:!0}))}function he(_){n.enabled===!1||n.enablePan===!1||le(_)}function Ne(_){switch(Oe(_),P.length){case 1:switch(n.touches.ONE){case zn.ROTATE:if(n.enableRotate===!1)return;ve(_),s=r.TOUCH_ROTATE;break;case zn.PAN:if(n.enablePan===!1)return;ge(_),s=r.TOUCH_PAN;break;default:s=r.NONE}break;case 2:switch(n.touches.TWO){case zn.DOLLY_PAN:if(n.enableZoom===!1&&n.enablePan===!1)return;xe(_),s=r.TOUCH_DOLLY_PAN;break;case zn.DOLLY_ROTATE:if(n.enableZoom===!1&&n.enableRotate===!1)return;we(_),s=r.TOUCH_DOLLY_ROTATE;break;default:s=r.NONE}break;default:s=r.NONE}s!==r.NONE&&n.dispatchEvent(ra)}function ce(_){switch(Oe(_),s){case r.TOUCH_ROTATE:if(n.enableRotate===!1)return;Xe(_),n.update();break;case r.TOUCH_PAN:if(n.enablePan===!1)return;Me(_),n.update();break;case r.TOUCH_DOLLY_PAN:if(n.enableZoom===!1&&n.enablePan===!1)return;Ie(_),n.update();break;case r.TOUCH_DOLLY_ROTATE:if(n.enableZoom===!1&&n.enableRotate===!1)return;Ae(_),n.update();break;default:s=r.NONE}}function ye(_){n.enabled!==!1&&_.preventDefault()}function Ge(_){P.push(_.pointerId)}function Re(_){delete Y[_.pointerId];for(let ne=0;ne<P.length;ne++)if(P[ne]==_.pointerId){P.splice(ne,1);return}}function pe(_){for(let ne=0;ne<P.length;ne++)if(P[ne]==_.pointerId)return!0;return!1}function Oe(_){let ne=Y[_.pointerId];ne===void 0&&(ne=new De,Y[_.pointerId]=ne),ne.set(_.pageX,_.pageY)}function Fe(_){const ne=_.pointerId===P[0]?P[1]:P[0];return Y[ne]}n.domElement.addEventListener("contextmenu",ye),n.domElement.addEventListener("pointerdown",Ve),n.domElement.addEventListener("pointercancel",y),n.domElement.addEventListener("wheel",ae,{passive:!1}),n.domElement.getRootNode().addEventListener("keydown",be,{passive:!0,capture:!0}),this.update()}}const ov="cabin_frame",lr="Scepter_depth_frame",lv="gripper_frame";function cr(i){const e=new Lt;e.setAttribute("position",new ft([],3));const t=new su({color:i,size:.035,transparent:!0,opacity:.78,sizeAttenuation:!0});return new Q_(e,t)}function cv(i){return new G(Number((i==null?void 0:i.x)||0),Number((i==null?void 0:i.y)||0),Number((i==null?void 0:i.z)||0))}function ou(i,e,t=new Map,n=new Set){if(t.has(i))return t.get(i);if(i===ov){const l={position:new G(0,0,0),quaternion:new Gt};return t.set(i,l),l}if(n.has(i))return null;const r=e.get(i);if(!r)return null;n.add(i);const s=ou(r.parentFrame,e,t,n)||{position:new G(0,0,0),quaternion:new Gt},o=r.position.clone().applyQuaternion(s.quaternion),a={position:s.position.clone().add(o),quaternion:s.quaternion.clone().multiply(r.quaternion.clone())};return t.set(i,a),n.delete(i),a}class uv{constructor({container:e}){this.container=e,this.layerState={},this.transformMap=new Map,this.cachedWorldTransforms=new Map,this.sourcePointCloudPositions={filteredWorldCoord:new Float32Array,rawWorldCoord:new Float32Array},this.pointCounts={filteredWorldCoord:0,rawWorldCoord:0,tiePoints:0,planningPoints:0},this.scene=new Z_,this.scene.background=new He(462872),this.scene.up.set(0,0,1),this.camera=new Rt(55,1,.01,200),this.camera.up.set(0,0,1),this.camera.position.set(2.2,-2.6,1.7),this.renderer=new K_({antialias:!0,alpha:!0}),this.renderer.setPixelRatio(window.devicePixelRatio||1),this.container.appendChild(this.renderer.domElement),this.controls=new av(this.camera,this.renderer.domElement),this.controls.enableDamping=!0,this.controls.target.set(0,0,.3),this.ambientLight=new iv(16777215,1.1),this.keyLight=new nv(16777215,.9),this.keyLight.position.set(3,-2,4),this.scene.add(this.ambientLight,this.keyLight),this.grid=new rv(8,16,2509919,1454141),this.grid.rotation.x=Math.PI/2,this.scene.add(this.grid),this.cabinAxes=new ia(.4),this.scene.add(this.cabinAxes),this.scepterFrame=new hn,this.scepterFrame.add(new ia(.28)),this.scene.add(this.scepterFrame),this.gripperFrame=new hn,this.gripperFrame.add(new ia(.22)),this.scene.add(this.gripperFrame),this.robotGroup=new hn;const t=new Jl({color:5015551,metalness:.1,roughness:.52,transparent:!0,opacity:.42}),n=new Ot(new Dn(.07,.07,.07),t);this.robotGroup.add(n),this.scene.add(this.robotGroup),this.tcpToolGroup=new hn;const r=new Jl({color:16747597,metalness:.08,roughness:.4,transparent:!0,opacity:.78}),s=new Ot(new Dn(.02,.01,.02),r);this.tcpToolGroup.add(s),this.scene.add(this.tcpToolGroup),this.filteredPointCloud=cr(6477567),this.rawPointCloud=cr(5204861),this.tiePoints=cr(5767069),this.planningPoints=cr(16307298),this.scene.add(this.filteredPointCloud,this.rawPointCloud,this.tiePoints,this.planningPoints),this.viewMode="camera",this.followCamera=!1,this.needsViewReset=!0,this.resizeObserver=new ResizeObserver(()=>this.resize()),this.resizeObserver.observe(this.container),this.resize(),this.startRenderLoop()}startRenderLoop(){const e=()=>{this.controls.update(),this.followCamera&&this.resetView(this.viewMode),this.needsViewReset&&(this.resetView(this.viewMode),this.needsViewReset=!1),this.renderer.render(this.scene,this.camera),this.rafId=window.requestAnimationFrame(e)};e()}resize(){const e=Math.max(this.container.clientWidth,280),t=Math.max(this.container.clientHeight,220);this.camera.aspect=e/t,this.camera.updateProjectionMatrix(),this.renderer.setSize(e,t)}setLayerState(e){this.layerState={...e},this.robotGroup.visible=!!e.showRobot,this.tcpToolGroup.visible=!!e.showRobot;const t=!!e.showAxes;this.cabinAxes.visible=t,this.scepterFrame.visible=t,this.gripperFrame.visible=t,this.grid.visible=t,this.filteredPointCloud.visible=!!e.showPointCloud&&e.pointCloudSource==="filteredWorldCoord",this.rawPointCloud.visible=!!e.showPointCloud&&e.pointCloudSource==="rawWorldCoord",this.tiePoints.visible=!!e.showTiePoints,this.planningPoints.visible=!!e.showPlanningMarkers,[this.filteredPointCloud,this.rawPointCloud,this.tiePoints,this.planningPoints].forEach(n=>{n.material.size=Number(e.pointSize)||.035,n.material.opacity=Number(e.pointOpacity)||.78,n.material.needsUpdate=!0})}setViewMode(e){this.viewMode=e,this.needsViewReset=!0}setFollowCamera(e){this.followCamera=!!e,this.needsViewReset=!0}resetView(e=this.viewMode){const t=this.getWorldTransform(lr);if(e==="camera"&&t){const s=new G(-1.1,-1.6,.8).applyQuaternion(t.quaternion),o=new G(0,0,.2).applyQuaternion(t.quaternion),a=t.position.clone().add(s),l=t.position.clone().add(o);this.camera.position.lerp(a,.28),this.controls.target.lerp(l,.28);return}const n=new G(2.6,-3.2,2.1),r=new G(0,0,.2);this.camera.position.lerp(n,.28),this.controls.target.lerp(r,.28)}handleTfMessage(e){(Array.isArray(e==null?void 0:e.transforms)?e.transforms:[]).forEach(n=>{var o,a,l,c,u,h,d,f,g,x,m,p,w,S,A;const r=n==null?void 0:n.child_frame_id,s=(o=n==null?void 0:n.header)==null?void 0:o.frame_id;!r||!s||this.transformMap.set(r,{parentFrame:s,position:new G(Number(((l=(a=n.transform)==null?void 0:a.translation)==null?void 0:l.x)||0),Number(((u=(c=n.transform)==null?void 0:c.translation)==null?void 0:u.y)||0),Number(((d=(h=n.transform)==null?void 0:h.translation)==null?void 0:d.z)||0)),quaternion:new Gt(Number(((g=(f=n.transform)==null?void 0:f.rotation)==null?void 0:g.x)||0),Number(((m=(x=n.transform)==null?void 0:x.rotation)==null?void 0:m.y)||0),Number(((w=(p=n.transform)==null?void 0:p.rotation)==null?void 0:w.z)||0),Number(((A=(S=n.transform)==null?void 0:S.rotation)==null?void 0:A.w)||1))})}),this.cachedWorldTransforms.clear(),this.applyFrameTransforms(),this.refreshPointCloudWorldPositions()}applyFrameTransforms(){this.applyGroupTransform(this.scepterFrame,this.getWorldTransform(lr));const e=this.getWorldTransform(lv);this.applyGroupTransform(this.gripperFrame,e);const t=this.getWorldTransform(lr);t&&(this.robotGroup.position.copy(t.position),this.robotGroup.quaternion.copy(t.quaternion)),e?(this.tcpToolGroup.visible=this.layerState.showRobot!==!1,this.tcpToolGroup.position.copy(e.position),this.tcpToolGroup.quaternion.copy(e.quaternion)):this.tcpToolGroup.visible=!1}applyGroupTransform(e,t){if(!t){e.visible=!1;return}this.layerState.showAxes!==!1&&(e.visible=!0),e.position.copy(t.position),e.quaternion.copy(t.quaternion)}getWorldTransform(e){return ou(e,this.transformMap,this.cachedWorldTransforms)}getKnownTransformCount(){return this.transformMap.size}refreshPointCloudWorldPositions(){this.applyPointCloudWorldPositions("filteredWorldCoord"),this.applyPointCloudWorldPositions("rawWorldCoord")}applyPointCloudWorldPositions(e){const t=this.sourcePointCloudPositions[e];if(!(t!=null&&t.length))return;const n=Float32Array.from(t),r=this.getWorldTransform(lr);if(r)for(let o=0;o<n.length;o+=3){const a=new G(n[o],n[o+1],n[o+2]);a.applyQuaternion(r.quaternion).add(r.position),n[o]=a.x,n[o+1]=a.y,n[o+2]=a.z}const s=e==="rawWorldCoord"?this.rawPointCloud:this.filteredPointCloud;s.geometry.setAttribute("position",new ft(n,3)),s.geometry.computeBoundingSphere()}setTiePointsMessage(e){const t=Array.isArray(e==null?void 0:e.PointCoordinatesArray)?e.PointCoordinatesArray:[],n=[];return t.forEach(r=>{const s=Array.isArray(r==null?void 0:r.World_coord)?r.World_coord:[];if(s.length<3)return;const o=cv({x:Number(s[0])/1e3,y:Number(s[1])/1e3,z:Number(s[2])/1e3});n.push(o.x,o.y,o.z)}),this.tiePoints.geometry.setAttribute("position",new ft(n,3)),this.tiePoints.geometry.computeBoundingSphere(),this.pointCounts.tiePoints=n.length/3,this.pointCounts.tiePoints}setPlanningMarkersMessage(e){const t=Array.isArray(e==null?void 0:e.markers)?e.markers:[],n=[];return t.forEach(r=>{(Array.isArray(r==null?void 0:r.points)?r.points:[]).forEach(o=>{n.push(Number((o==null?void 0:o.x)||0),Number((o==null?void 0:o.y)||0),Number((o==null?void 0:o.z)||0))})}),this.planningPoints.geometry.setAttribute("position",new ft(n,3)),this.planningPoints.geometry.computeBoundingSphere(),this.pointCounts.planningPoints=n.length/3,this.pointCounts.planningPoints}setPointCloudImageMessage(e,t){const{positions:n,count:r}=Mh(t,{sampleStep:e==="rawWorldCoord"?8:5,maxPoints:e==="rawWorldCoord"?12e3:18e3});return this.sourcePointCloudPositions[e]=n,this.applyPointCloudWorldPositions(e),this.pointCounts[e]=r,r}}class hv{constructor({canvas:e,overlayCanvas:t,onSelectionChanged:n,onMessage:r}){this.canvas=e,this.overlayCanvas=t,this.ctx=e.getContext("2d"),this.overlayCtx=t.getContext("2d"),this.onSelectionChanged=n,this.onMessage=r,this.lastImageMessage=null,this.lastS2ResultMessage=null,this.lastExecutionResultMessage=null,this.overlaySource="s2",this.savedWorkspacePoints=[],this.selectedPoints=[],this.displaySettings={mode:"auto",gamma:.85,overlayOpacity:.88},this.dragState={activeIndex:-1,moved:!1},this.suppressNextCanvasClick=!1}bindPointerEvents(){this.canvas.addEventListener("click",e=>this.handleCanvasClick(e)),this.canvas.addEventListener("pointerdown",e=>this.handlePointerDown(e)),this.canvas.addEventListener("pointermove",e=>this.handlePointerMove(e)),this.canvas.addEventListener("pointerup",()=>this.handlePointerUp()),this.canvas.addEventListener("pointerleave",()=>this.handlePointerUp())}setDisplaySettings(e){this.displaySettings={...this.displaySettings,...e},this.overlayCanvas.style.opacity=String(this.displaySettings.overlayOpacity),this.draw()}setBaseImageMessage(e){this.lastImageMessage=e,this.draw()}setS2OverlayMessage(e){this.lastS2ResultMessage=e,this.overlaySource!=="execution"&&(this.overlaySource="s2"),this.drawOverlay()}setExecutionOverlayMessage(e){this.lastExecutionResultMessage=e,this.overlaySource==="execution"&&this.drawOverlay()}setOverlaySource(e){this.overlaySource=e,this.drawOverlay()}setSavedWorkspacePoints(e){this.savedWorkspacePoints=Array.isArray(e)?e:[],this.draw()}setSavedWorkspacePayload(e){this.savedWorkspacePoints=mh(e),this.draw()}getSelectedPoints(){return[...this.selectedPoints]}getSavedWorkspacePoints(){return[...this.savedWorkspacePoints]}clearSelection(){this.selectedPoints=[],this.notifySelectionChanged(),this.draw()}undoSelection(){this.selectedPoints.length&&(this.selectedPoints=this.selectedPoints.slice(0,-1),this.notifySelectionChanged(),this.draw())}buildWorkspacePayload(){return yc(this.selectedPoints)}notifySelectionChanged(){var e;(e=this.onSelectionChanged)==null||e.call(this,this.getSelectedPoints())}draw(){if(!this.lastImageMessage)return;const e=bo(this.lastImageMessage,this.displaySettings);this.canvas.width=e.width,this.canvas.height=e.height,this.overlayCanvas.width=e.width,this.overlayCanvas.height=e.height,this.ctx.putImageData(e,0,0),this.drawWorkspacePolylines(),this.drawOverlay()}drawOverlay(){const e=this.overlaySource==="execution"?this.lastExecutionResultMessage:this.lastS2ResultMessage;if(this.overlayCtx.clearRect(0,0,this.overlayCanvas.width,this.overlayCanvas.height),!e)return;const t=bo(e,{mode:"raw",gamma:1});this.overlayCanvas.width=t.width,this.overlayCanvas.height=t.height,this.overlayCtx.putImageData(t,0,0),this.overlayCanvas.style.opacity=String(this.displaySettings.overlayOpacity)}drawWorkspacePolylines(){if(this.ctx.save(),this.ctx.lineWidth=2,this.ctx.font="18px monospace",this.savedWorkspacePoints.length>=2){this.ctx.strokeStyle="#6aa6ff",this.ctx.fillStyle="#6aa6ff",this.ctx.setLineDash([10,6]),this.ctx.beginPath(),this.ctx.moveTo(this.savedWorkspacePoints[0].x,this.savedWorkspacePoints[0].y);for(let e=1;e<this.savedWorkspacePoints.length;e+=1)this.ctx.lineTo(this.savedWorkspacePoints[e].x,this.savedWorkspacePoints[e].y);this.savedWorkspacePoints.length===4&&this.ctx.closePath(),this.ctx.stroke(),this.ctx.setLineDash([])}if(this.selectedPoints.length){if(this.ctx.strokeStyle="#4de3a5",this.ctx.fillStyle="#ffae42",this.selectedPoints.length>=2){this.ctx.beginPath(),this.ctx.moveTo(this.selectedPoints[0].x,this.selectedPoints[0].y);for(let e=1;e<this.selectedPoints.length;e+=1)this.ctx.lineTo(this.selectedPoints[e].x,this.selectedPoints[e].y);this.selectedPoints.length===4&&this.ctx.closePath(),this.ctx.stroke()}this.selectedPoints.forEach((e,t)=>{this.ctx.beginPath(),this.ctx.arc(e.x,e.y,5,0,Math.PI*2),this.ctx.fill(),this.ctx.strokeStyle="#ffffff",this.ctx.stroke(),this.ctx.fillStyle="#4de3a5",this.ctx.fillText(`${t+1}`,e.x+8,e.y-8),this.ctx.fillStyle="#ffae42"})}this.ctx.restore()}handleCanvasClick(e){var n,r;if(this.suppressNextCanvasClick){this.suppressNextCanvasClick=!1;return}if(!this.lastImageMessage){(n=this.onMessage)==null||n.call(this,"IR 图像还没到，先等一帧");return}if(this.selectedPoints.length>=4){(r=this.onMessage)==null||r.call(this,"已经点满 4 个角点了，先清空或撤销再继续");return}const t=Ms({clientX:e.clientX,clientY:e.clientY,rect:this.canvas.getBoundingClientRect(),imageWidth:Number(this.lastImageMessage.width),imageHeight:Number(this.lastImageMessage.height)});this.selectedPoints=[...this.selectedPoints,t],this.notifySelectionChanged(),this.draw()}handlePointerDown(e){var r,s;if(!this.lastImageMessage||!this.selectedPoints.length)return;const t=Ms({clientX:e.clientX,clientY:e.clientY,rect:this.canvas.getBoundingClientRect(),imageWidth:Number(this.lastImageMessage.width),imageHeight:Number(this.lastImageMessage.height)}),n=gh(this.selectedPoints,t,12);n<0||(this.dragState={activeIndex:n,moved:!1},(s=(r=this.canvas).setPointerCapture)==null||s.call(r,e.pointerId))}handlePointerMove(e){if(this.dragState.activeIndex<0||!this.lastImageMessage)return;const t=Ms({clientX:e.clientX,clientY:e.clientY,rect:this.canvas.getBoundingClientRect(),imageWidth:Number(this.lastImageMessage.width),imageHeight:Number(this.lastImageMessage.height)});this.selectedPoints=_h(this.selectedPoints,this.dragState.activeIndex,t),this.dragState={...this.dragState,moved:!0},this.draw()}handlePointerUp(){if(this.dragState.activeIndex<0)return;const e=this.dragState.moved;this.dragState={activeIndex:-1,moved:!1},e&&(this.suppressNextCanvasClick=!0,this.notifySelectionChanged())}}class dv{constructor(e){this.rootElement=e,this.logs=[],this.displaySettings=Dh(),this.ui=new Uh(e),this.ui.renderShell(),this.panelManager=new Rh,this.panelManager.init(e),e.querySelectorAll(".floating-panel").forEach(n=>{this.panelManager.registerPanel(n)});const t=this.ui.getCanvasRefs();this.workspaceView=new hv({canvas:t.canvas,overlayCanvas:t.overlayCanvas,onSelectionChanged:n=>{this.ui.renderPointList(n),this.refreshActionState()},onMessage:n=>this.addLog(n,"info")}),this.workspaceView.bindPointerEvents(),this.workspaceView.setDisplaySettings(this.displaySettings),this.ui.setDisplaySettings(this.displaySettings),this.ui.renderPointList([]),this.sceneView=new uv({container:this.ui.getSceneContainer()}),this.topicLayerController=new Ch({ui:this.ui,sceneView:this.sceneView,callbacks:{onLog:(n,r)=>this.addLog(n,r)}}),this.topicLayerController.init(),this.statusMonitorController=new fh({onStatusChip:(n,r,s)=>this.ui.setStatusChipState(n,r,s),onLog:(n,r)=>this.addLog(n,r)}),this.rosConnectionController=new hh({onConnectionInfo:(n,r,s)=>{this.ui.setConnectionInfo(n,r,s),this.statusMonitorController.setConnectionState(s,r)},onRosReady:n=>{this.statusMonitorController.start(n.ros),this.refreshActionState()},onRosUnavailable:()=>{var n;this.statusMonitorController.stop(),(n=this.legacyCommandController)==null||n.reset(),this.refreshActionState()},onLog:(n,r)=>this.addLog(n,r),onBaseImage:n=>{this.workspaceView.setBaseImageMessage(n),this.workspaceView.getSelectedPoints().length===0&&this.ui.setResultMessage("IR 图像已就绪，直接在图上点 4 个角点。")},onSavedWorkspacePayload:n=>{this.workspaceView.setSavedWorkspacePayload(n),this.refreshActionState()},onS2Overlay:n=>{this.workspaceView.setS2OverlayMessage(n),this.ui.setResultMessage("已收到最新 S2 result，当前覆盖层已叠加到 IR 底图。")},onExecutionOverlay:n=>{this.workspaceView.setExecutionOverlayMessage(n),this.workspaceView.overlaySource==="execution"&&this.ui.setResultMessage("执行层 result_img 已更新，当前覆盖层正在显示 /pointAI/result_image_raw。")},onPointCloudImage:(n,r)=>{const s=this.sceneView.setPointCloudImageMessage(n,r);this.topicLayerController.updateStats({[`${n}Count`]:s})},onTiePoints:n=>{const r=this.sceneView.setTiePointsMessage(n);this.topicLayerController.updateStats({tiePointCount:r})},onPlanningMarkers:n=>{const r=this.sceneView.setPlanningMarkersMessage(n);this.topicLayerController.updateStats({planningPointCount:r})},onTfMessage:n=>{this.sceneView.handleTfMessage(n),this.topicLayerController.updateStats({tfFrameCount:this.sceneView.getKnownTransformCount()})}}),this.taskActionController=new bh({rosConnection:this.rosConnectionController,workspaceView:this.workspaceView,callbacks:{onResultMessage:n=>this.ui.setResultMessage(n),onLog:(n,r)=>this.addLog(n,r)}}),this.legacyCommandController=new Hu({rosConnection:this.rosConnectionController,callbacks:{onResultMessage:n=>this.ui.setResultMessage(n),onLog:(n,r)=>this.addLog(n,r)}})}init(){return this.bindUIEvents(),this.refreshActionState(),this.syncToolbarState(),this.rosConnectionController.connect(),this}bindUIEvents(){this.ui.onToolbarAction(e=>{this.handleToolbarAction(e)}),this.ui.onTaskAction(e=>{this.taskActionController.handle(e),this.refreshActionState()}),this.ui.onWorkspaceAction(e=>{e==="undo"?this.workspaceView.undoSelection():e==="clear"&&this.workspaceView.clearSelection(),this.refreshActionState()}),this.ui.onDisplaySettingsChange(e=>{this.displaySettings={mode:e.mode,gamma:Number.isFinite(e.gamma)?e.gamma:.85,overlayOpacity:Number.isFinite(e.overlayOpacity)?e.overlayOpacity:.88},Ih(this.displaySettings),this.workspaceView.setDisplaySettings(this.displaySettings),this.ui.setDisplaySettings(this.displaySettings)}),this.ui.onSceneControlsChange(e=>{this.topicLayerController.handleSceneControlsChange(e),this.syncToolbarState()}),this.ui.onTopicLayerControlsChange(e=>{this.topicLayerController.handleLayerControlsChange(e),this.syncToolbarState()}),this.ui.onLegacyCommand(e=>{this.legacyCommandController.handle(e,this.ui.getParameterValues())}),this.ui.onClearLogs(()=>{this.logs=[],this.ui.renderLogs(this.logs)})}handleToolbarAction(e){if(!e)return;if(e.startsWith("toggle-panel:")){const n=e.split(":")[1];this.ui.togglePanelVisible(n);return}const t=this.topicLayerController.getState();switch(e){case"view:camera":this.topicLayerController.handleSceneControlsChange({viewMode:"camera"});break;case"view:global":this.topicLayerController.handleSceneControlsChange({viewMode:"global"});break;case"toggle-follow-camera":this.topicLayerController.handleSceneControlsChange({followCamera:!t.followCamera});break;case"toggle-layer:pointCloud":this.topicLayerController.handleLayerControlsChange({showPointCloud:!t.showPointCloud});break;case"toggle-layer:tiePoints":this.topicLayerController.handleLayerControlsChange({showTiePoints:!t.showTiePoints});break;default:return}this.syncToolbarState()}syncToolbarState(){this.ui.setToolbarSceneState(this.topicLayerController.getState())}refreshActionState(){const e=this.rosConnectionController.isReady(),t=this.rosConnectionController.getResources(),n=this.workspaceView.getSelectedPoints(),r=this.workspaceView.getSavedWorkspacePoints();this.ui.setTaskButtonsEnabled({submitQuad:e&&!!(t!=null&&t.workspaceQuadPublisher&&(t!=null&&t.runWorkspaceS2Publisher))&&n.length===4,runSavedS2:e&&!!(t!=null&&t.runWorkspaceS2Publisher)&&r.length===4,scanPlan:e&&!!(t!=null&&t.startPseudoSlamScanActionClient),startExecution:e&&!!(t!=null&&t.executionModeService)&&!!(t!=null&&t.startGlobalWorkActionClient),startExecutionKeepMemory:e&&!!(t!=null&&t.executionModeService)&&!!(t!=null&&t.startGlobalWorkActionClient),runBindPathTest:e&&!!(t!=null&&t.runDirectBindPathTestActionClient)}),this.ui.setWorkspaceButtonsEnabled({undo:n.length>0,clear:n.length>0})}addLog(e,t="info"){const n=new Date().toLocaleTimeString("zh-CN",{hour12:!1});this.logs=[{timestamp:n,message:e,level:t},...this.logs].slice(0,80),this.ui.renderLogs(this.logs)}}const fv=document.getElementById("app"),lu=new dv(fv);window.tieRobotFrontApp=lu;lu.init();
