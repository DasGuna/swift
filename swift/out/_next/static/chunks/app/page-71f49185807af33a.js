(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{3751:function(e,t,s){Promise.resolve().then(s.bind(s,883)),Promise.resolve().then(s.bind(s,7536)),Promise.resolve().then(s.t.bind(s,2506,23))},883:function(e,t,s){"use strict";var r=s(7437),n=s(5452);t.default=function(){let{name:e}=(0,n.M4)({name:"World"});return(0,r.jsx)(r.Fragment,{children:(0,r.jsx)(n.Zf,{oneLineLabels:!0,collapsed:!0})})}},7536:function(e,t,s){"use strict";s.d(t,{default:function(){return F}});var r=s(7437),n=s(7776),a=s(3149),o=s(2265),l=s(6547),i=s(9449),c=s.n(i),u=s(2708),m=s(5533);let d=e=>{let{viewport:t,set:s}=(0,u.A)(),{width:n,height:a}=t,l=(0,o.useRef)(null);return(0,r.jsx)(m.c,{makeDefault:!0,ref:l,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:a/n})},f=()=>{let{scene:e}=(0,u.A)();return e.background=new n.Color(7895160),e.fog=new n.Fog(.787878,50,60),(0,r.jsxs)("mesh",{receiveShadow:!0,children:[(0,r.jsx)("planeGeometry",{args:[200,200]}),(0,r.jsx)("meshPhongMaterial",{color:4934475,specular:new n.Color(1052688)})]})},h=e=>{let t=(0,o.useRef)(null);return(0,r.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var p=s(2615),x=s(2325),E=s(1957),j=s(7413),g=s(2851);let w=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{w(e,t,s)})},S=e=>{let t=(0,u.F)(j.j,e.url),s=(0,o.useMemo)(()=>t.clone(),[t]);return(0,r.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,r.jsx)("primitive",{object:s,attach:"geometry"}),(0,r.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},y=e=>(0,r.jsx)(o.Fragment,{children:(0,r.jsx)(E.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]]})}),v=e=>{let t=(0,u.F)(g.G,e.url),s=(0,o.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,o.useEffect)(()=>{s.children.forEach(t=>{w(t,e.opacity,!0)}),s.name="loaded"},[s,e.opacity]),(0,r.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var D=e=>{if(console.log("LOADER: received filename is: "+e.filename),null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,r.jsx)(r.Fragment,{});{let t=e.filename.split(".").pop(),s=e.filename;switch(console.log("LOADER: extension is: "+t),console.log("LOADER: url is: "+s),s="/retrieve/"+s,t){case"stl":return console.log("LOADING STL FILE HERE"),(0,r.jsx)(S,{url:s,...e});case"splat":return console.log("LOADING SPLAT FILE HERE"),(0,r.jsx)(y,{url:s,...e});default:return console.log("LOADING DEFAULT HERE"),(0,r.jsx)(v,{url:s,...e})}}};let b=e=>{let[t,s]=(0,o.useState)(!1);return(0,o.useEffect)(()=>{s(!0)},[]),(0,r.jsx)(o.Fragment,{children:t&&(0,r.jsx)(o.Suspense,{fallback:(0,r.jsx)(function(){let{progress:e}=(0,p.S)();return(0,r.jsxs)(x.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,r.jsx)(D,{...e})})})},L=e=>{let t=(0,o.useRef)(null);switch((0,o.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,r.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,r.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,r.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},C=e=>{let t=(0,o.useRef)(null);return(0,r.jsxs)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",children:[(0,r.jsx)(L,{...e}),(0,r.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})},O=e=>{if(!1==e.display)return(0,r.jsx)(o.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,r.jsx)(b,{...e});default:return(0,r.jsx)(C,{...e})}},_=e=>(0,r.jsx)("group",{children:e.meshes.map((e,t)=>(0,r.jsx)(O,{...e},t))}),A=o.forwardRef((e,t)=>(0,r.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,r.jsx)(_,{meshes:e},t))}));A.displayName="GroupCollection";var R=s(8592),T=function(){return(0,r.jsx)(R.G,{})};let N=new(s(6731)).EventEmitter;var q=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},r=[...e.formElements];return s[t.index]=t.data,r[t.index-3][t.valueName]=t.value,{formElements:r,formData:s};case"userInputNoState":let n={...e.formData};return n[t.index]=t.data,{formElements:[...e.formElements],formData:n};case"wsUpdate":let a=[...e.formElements];return a[t.index-3]=t.data,{...e,formElements:a};case"reset":let o={...e.formData};return t.indices.map(e=>{delete o[e]}),{formData:o,formElements:[...e.formElements]};default:throw Error()}};let k=e=>{let t=(0,o.useRef)(null),[s,i]=(0,o.useState)([]),[u,m]=(0,o.useState)(!1),[p,x]=(0,o.useState)(!1),[E,j]=(0,o.useReducer)(q,{formData:{},formElements:[]});(0,o.useEffect)(()=>{let t=!0;x(!0);let s=e.port,r=window.location.search.substring(1).split("&");if(0===s&&(console.log("WEBSOCKET: port is 0, using server_params..."),s=parseInt(r[0])),null===s||isNaN(s))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e=new WebSocket("ws://localhost:"+s+"/");console.log("WEBSOCKET: WebSocket using port: "+s),t&&(e.onopen=()=>{e.onclose=()=>{setTimeout(()=>{window.close()},5e3)},e.send("Connected"),m(!0)},N.on("wsSwiftTx",t=>{console.log("WEBSOCKET: sending back: "+t),e.send(t)})),e&&(e.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],r=t[1];N.emit("wsRx",s,r)})}},[e.port]);let g={shape:e=>{let t=s.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),i([...s,e]),N.emit("wsSwiftTx",t)},shape_mounted:e=>{let s=e[0],r=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+s);try{var n;let e=0;null===(n=t.current)||void 0===n||n.children[s].children.forEach((t,s)=>{"loaded"===t.name&&e++}),e===r?N.emit("wsSwiftTx","1"):N.emit("wsSwiftTx","0")}catch(e){N.emit("wsSwiftTx","0")}}};return(0,o.useEffect)(()=>{N.removeAllListeners("wsRx"),N.on("wsRx",(e,t)=>{console.log(e),g[e](t)})},[s,E,g]),n.Object3D.DEFAULT_UP=new n.Vector3(0,0,1),(0,r.jsx)("div",{className:c().visContainer,children:(0,r.jsxs)(a.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,r.jsx)(T,{}),(0,r.jsx)(d,{t:[1,1,1]}),(0,r.jsx)("hemisphereLight",{groundColor:new n.Color(1118498)}),(0,r.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(l.z,{panSpeed:.5,rotateSpeed:.4}),(0,r.jsx)("axesHelper",{args:[100]}),(0,r.jsx)(f,{}),(0,r.jsx)(o.Suspense,{fallback:null,children:(0,r.jsx)(A,{meshes:s,ref:t})})]})})};k.displayName="Viewer";var F=k},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[308,689,678,971,23,744],function(){return e(e.s=3751)}),_N_E=e.O()}]);