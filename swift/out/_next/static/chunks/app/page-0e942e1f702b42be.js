(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{3751:function(e,t,s){Promise.resolve().then(s.bind(s,883)),Promise.resolve().then(s.bind(s,182)),Promise.resolve().then(s.t.bind(s,2506,23))},883:function(e,t,s){"use strict";var r=s(7437),n=s(5452);t.default=function(){let{name:e}=(0,n.M4)({name:"World"});return(0,r.jsx)(r.Fragment,{children:(0,r.jsx)(n.Zf,{oneLineLabels:!0,collapsed:!0})})}},182:function(e,t,s){"use strict";s.d(t,{default:function(){return T}});var r=s(7437),n=s(7776),o=s(3149),a=s(2265),l=s(6547),i=s(9449),c=s.n(i),u=s(2708),m=s(5533),f=s(2615),d=s(2325),h=s(1957),p=s(7413),x=s(2851);let j=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{j(e,t,s)})},E=e=>{let t=(0,u.F)(p.j,e.url),s=(0,a.useMemo)(()=>t.clone(),[t]);return(0,r.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,r.jsx)("primitive",{object:s,attach:"geometry"}),(0,r.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},w=e=>(0,r.jsx)(a.Fragment,{children:(0,r.jsx)(h.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]]})}),g=e=>{let t=(0,u.F)(x.G,e.url),s=(0,a.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,a.useEffect)(()=>{s.children.forEach(t=>{j(t,e.opacity,!0)}),s.name="loaded"},[s]),(0,r.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var S=e=>{let t=e.filename.split(".").pop().toLowerCase(),s=e.filename;switch(s="/retrieve/"+s,t){case"stl":return console.log("LOADING STL FILE HERE"),(0,r.jsx)(E,{url:s,...e});case"splat":return console.log("LOADING SPLAT FILE HERE"),(0,r.jsx)(w,{url:s,...e});default:return console.log("LOADING DEFAULT HERE"),(0,r.jsx)(g,{url:s,...e})}};let v=e=>{let{viewport:t,set:s}=(0,u.A)(),{width:n,height:o}=t,l=(0,a.useRef)(null);return(0,r.jsx)(m.c,{makeDefault:!0,ref:l,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:o/n})},y=()=>{let{scene:e}=(0,u.A)();return e.background=new n.Color(7895160),e.fog=new n.Fog(.787878,50,60),(0,r.jsxs)("mesh",{receiveShadow:!0,children:[(0,r.jsx)("planeGeometry",{args:[200,200]}),(0,r.jsx)("meshPhongMaterial",{color:4934475,specular:new n.Color(1052688)})]})},b=e=>{let t=(0,a.useRef)(null);return(0,r.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})},C=e=>{let[t,s]=(0,a.useState)(!1);return(0,a.useEffect)(()=>{s(!0)},[]),(0,r.jsx)(a.Fragment,{children:t&&(0,r.jsx)(a.Suspense,{fallback:(0,r.jsx)(function(){let{active:e,progress:t,errors:s,item:n,loaded:o,total:a}=(0,f.S)();return(0,r.jsxs)(d.V,{center:!0,children:[Math.round(t)," % loaded"]})},{}),children:(0,r.jsx)(S,{...e})})})},L=e=>{let t=(0,a.useRef)(null);return(0,r.jsx)("mesh",{ref:t,position:[0,0,0],quaternion:[0,0,0,1],castShadow:!0,name:"loaded",children:(0,r.jsx)("boxBufferGeometry",{args:[1,1,1]})})},_=e=>{if(!1==e.display)return(0,r.jsx)(a.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,r.jsx)(C,{...e});default:return(0,r.jsx)(L,{...e})}};var D=s(8592),N=function(){return(0,r.jsx)(D.G,{})};let A=new(s(6731)).EventEmitter;var k=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},r=[...e.formElements];return s[t.index]=t.data,r[t.index-3][t.valueName]=t.value,{formElements:r,formData:s};case"userInputNoState":let n={...e.formData};return n[t.index]=t.data,{formElements:[...e.formElements],formData:n};case"wsUpdate":let o=[...e.formElements];return o[t.index-3]=t.data,{...e,formElements:o};case"reset":let a={...e.formData};return t.indices.map(e=>{delete a[e]}),{formData:a,formElements:[...e.formElements]};default:throw Error()}};let R=e=>(0,r.jsx)("group",{children:e.meshes.map((e,t)=>(0,r.jsx)(_,{...e},t))}),F=a.forwardRef((e,t)=>(0,r.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,r.jsx)(R,{meshes:e},t))}));F.displayName="GroupCollection";let O=e=>{let t=(0,a.useRef)(null),[s,i]=(0,a.useState)([]),[u,m]=(0,a.useState)(!1),[f,d]=(0,a.useState)(!1),[h,p]=(0,a.useReducer)(k,{formData:{},formElements:[]});return(0,a.useEffect)(()=>{let t=!0;d(!0);let s=e.port,r=window.location.search.substring(1).split("&");if(console.log(r),0===s&&(console.log("WEBSOCKET: port is 0, using server_params..."),s=parseInt(r[0])),null===s||isNaN(s))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e=new WebSocket("ws://localhost:"+s+"/");console.log("WEBSOCKET: WebSocket using port: "+s),t&&(e.onopen=()=>{e.onclose=()=>{setTimeout(()=>{window.close()},5e3)},e.send("Connected"),m(!0)},A.on("wsSwiftTx",t=>{console.log(t),e.send(t)})),e&&(e.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],r=t[1];A.emit("wsRx",s,r)})}},[e.port]),(0,a.useEffect)(()=>{A.removeAllListeners("wsRx"),A.on("wsRx",(e,t)=>{console.log(e)})},[s,h,{shape:e=>{let t=s.length.toString();console.log(t),i([...s,e]),A.emit("wsSwiftTx",t)}}]),n.Object3D.DEFAULT_UP=new n.Vector3(0,0,1),(0,r.jsx)("div",{className:c().visContainer,children:(0,r.jsxs)(o.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,r.jsx)(N,{}),(0,r.jsx)(v,{t:[1,1,1]}),(0,r.jsx)("hemisphereLight",{groundColor:new n.Color(1118498)}),(0,r.jsx)(b,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(b,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(l.z,{panSpeed:.5,rotateSpeed:.4}),(0,r.jsx)("axesHelper",{args:[100]}),(0,r.jsx)(y,{}),(0,r.jsx)(a.Suspense,{fallback:null,children:(0,r.jsx)(F,{meshes:s,ref:t})})]})})};O.displayName="Viewer";var T=O},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[308,689,678,971,23,744],function(){return e(e.s=3751)}),_N_E=e.O()}]);