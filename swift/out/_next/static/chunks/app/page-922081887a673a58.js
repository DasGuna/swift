(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{1338:function(e,t,s){Promise.resolve().then(s.bind(s,4751)),Promise.resolve().then(s.bind(s,7536)),Promise.resolve().then(s.t.bind(s,2506,23))},4751:function(e,t,s){"use strict";var o=s(7437),r=s(2265),n=s(8154),a=s(2888);t.default=function(){let e=(0,a.o)(e=>e.objects),t=(0,a.o)(e=>e.targetID),s=(0,a.o)(e=>e.noObjects),l=(0,a.o)(e=>e.setObjectVisibility),i=(0,a.o)(e=>e.setTargetID),c=(0,a.o)(e=>e.setPos),u=(0,a.o)(e=>e.list);(0,r.useEffect)(()=>{m({NoObjects:s}),m({Visible:e[t][0].display}),m({ID:t})},[s,t,u]);let[d,m]=(0,n.M4)("Object",()=>({ID:{value:t,onChange:e=>{i(e)}},Visible:{value:!0,onChange:e=>l(e)},Position:{value:{x:0,y:0,z:0},onChange:e=>c(e)},NoObjects:{label:"Total:",value:s}}));return(0,o.jsx)(o.Fragment,{})}},2888:function(e,t,s){"use strict";s.d(t,{o:function(){return a}});var o=s(9099),r=s(2844);let n={objects:[[{id:0,stype:"other",scale:[1,1,1],filename:void 0,radius:0,length:0,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:"hotpink",opacity:0,display:!1,head_length:0,head_radius:0}]],noObjects:1,targetID:0,list:[0]},a=(0,o.Ue)((0,r.n)(e=>({...n,setObjectVisibility:t=>e(e=>{e.objects[e.targetID][0].display=t}),setTargetID:t=>e(e=>{t<e.noObjects?e.targetID=t:(e.targetID=e.targetID,alert("Invalid Target ID"))}),addObject:t=>e(e=>{e.list=[...e.list,e.noObjects],e.noObjects+=1,e.objects=[...e.objects,t],console.log("STORE: addObject"),console.log(e.list),console.log(e.objects),console.log("STORE: addObject - END")}),setPos:t=>e(e=>{e.objects[e.targetID][0].t=[t.x,t.y,t.z]})})))},7536:function(e,t,s){"use strict";s.d(t,{default:function(){return F}});var o=s(7437),r=s(7776),n=s(3149),a=s(2265),l=s(6547),i=s(9449),c=s.n(i),u=s(9367),d=s(5533);let m=e=>{let{viewport:t,set:s}=(0,u.A)(),{width:r,height:n}=t,l=(0,a.useRef)(null);return(0,o.jsx)(d.c,{makeDefault:!0,ref:l,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/r})},f=()=>{let{scene:e}=(0,u.A)();return e.background=new r.Color(7895160),e.fog=new r.Fog(.787878,50,60),(0,o.jsxs)("mesh",{receiveShadow:!0,children:[(0,o.jsx)("planeGeometry",{args:[200,200]}),(0,o.jsx)("meshPhongMaterial",{color:4934475,specular:new r.Color(1052688)})]})},h=e=>{let t=(0,a.useRef)(null);return(0,o.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var p=s(7581),j=s(2325),x=s(1957),g=s(7413),E=s(2851);let b=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{b(e,t,s)})},y=e=>{let t=(0,u.F)(g.j,e.url),s=(0,a.useMemo)(()=>t.clone(),[t]);return(0,o.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,o.jsx)("primitive",{object:s,attach:"geometry"}),(0,o.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},w=e=>(0,o.jsx)(a.Fragment,{children:(0,o.jsx)(x.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],alphaTest:.1})}),S=e=>{let t=(0,u.F)(E.G,e.url),s=(0,a.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,a.useEffect)(()=>{s.children.forEach(t=>{b(t,e.opacity,!0)}),s.name="loaded"},[s,e.opacity]),(0,o.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var v=e=>{if(console.log("LOADER: received filename is: "+e.filename),null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,o.jsx)(o.Fragment,{});{let t=e.filename.split(".").pop(),s=e.filename;switch(console.log("LOADER: extension is: "+t),console.log("LOADER: url is: "+s),s="/retrieve/"+s,null==t?void 0:t.toLowerCase()){case"stl":return console.log("LOADING STL FILE HERE"),(0,o.jsx)(y,{url:s,...e});case"splat":return console.log("LOADING SPLAT FILE HERE"),(0,o.jsx)(w,{url:s,...e});default:return console.log("LOADING DEFAULT HERE"),(0,o.jsx)(S,{url:s,...e})}}};let D=e=>{let[t,s]=(0,a.useState)(!1);return(0,a.useEffect)(()=>{s(!0)},[]),(0,o.jsx)(a.Fragment,{children:t&&(0,o.jsx)(a.Suspense,{fallback:(0,o.jsx)(function(){let{progress:e}=(0,p.S)();return(0,o.jsxs)(j.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,o.jsx)(v,{...e})})})},O=e=>{let t=(0,a.useRef)(null);switch((0,a.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,o.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,o.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,o.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},T=e=>{let t=(0,a.useRef)(null);return(0,o.jsxs)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",onClick:()=>{console.log(e.id)},children:[(0,o.jsx)(O,{...e}),(0,o.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})},C=e=>{if(!1==e.display)return(0,o.jsx)(a.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,o.jsx)(D,{...e});default:return(0,o.jsx)(T,{...e})}},I=e=>(0,o.jsx)("group",{children:e.meshes.map((e,t)=>(0,o.jsx)(C,{...e},t))}),R=a.forwardRef((e,t)=>(0,o.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,o.jsx)(I,{meshes:e},t))}));R.displayName="GroupCollection";var L=s(8592),N=function(){return(0,o.jsx)(L.G,{})};let _=new(s(6731)).EventEmitter;var A=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},o=[...e.formElements];return s[t.index]=t.data,o[t.index-3][t.valueName]=t.value,{formElements:o,formData:s};case"userInputNoState":let r={...e.formData};return r[t.index]=t.data,{formElements:[...e.formElements],formData:r};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let a={...e.formData};return t.indices.map(e=>{delete a[e]}),{formData:a,formElements:[...e.formElements]};default:throw Error()}},k=s(2888);let q=e=>{let t=(0,a.useRef)(null),[s,i]=(0,a.useState)([]),[u,d]=(0,a.useState)(!1),[p,j]=(0,a.useState)(!1),[x,g]=(0,a.useReducer)(A,{formData:{},formElements:[]}),E=(0,k.o)(e=>e.addObject),b=(0,k.o)(e=>e.objects);(0,a.useEffect)(()=>{console.log("CURRENT DEFAULT STORE"),console.log(b);let t=!0;j(!0);let s=e.port,o=window.location.search.substring(1).split("&");if(0===s&&(console.log("WEBSOCKET: port is 0, using server_params..."),s=parseInt(o[0])),null===s||isNaN(s))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e=new WebSocket("ws://localhost:"+s+"/");console.log("WEBSOCKET: WebSocket using port: "+s),t&&(e.onopen=()=>{e.onclose=()=>{setTimeout(()=>{window.close()},5e3)},e.send("Connected"),d(!0)},_.on("wsSwiftTx",t=>{console.log("WEBSOCKET: sending back: "+t),e.send(t)})),e&&(e.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],o=t[1];_.emit("wsRx",s,o)})}},[e.port]);let y={shape:e=>{let t=s.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),E(e),_.emit("wsSwiftTx",t)},shape_mounted:e=>{let s=e[0],o=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+s);try{var r;let e=0;null===(r=t.current)||void 0===r||r.children[s].children.forEach((t,s)=>{"loaded"===t.name&&e++}),e===o?_.emit("wsSwiftTx","1"):_.emit("wsSwiftTx","0")}catch(e){_.emit("wsSwiftTx","0")}}};return(0,a.useEffect)(()=>{_.removeAllListeners("wsRx"),_.on("wsRx",(e,t)=>{console.log(e),y[e](t)})},[s,x,y]),r.Object3D.DEFAULT_UP=new r.Vector3(0,0,1),(0,o.jsx)("div",{className:c().visContainer,children:(0,o.jsxs)(n.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,o.jsx)(N,{}),(0,o.jsx)(m,{t:[1,1,1]}),(0,o.jsx)("hemisphereLight",{groundColor:new r.Color(1118498)}),(0,o.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(l.z,{panSpeed:.5,rotateSpeed:.4}),(0,o.jsx)("axesHelper",{args:[100]}),(0,o.jsx)(f,{}),(0,o.jsx)(a.Suspense,{fallback:null,children:(0,o.jsx)(R,{meshes:b,ref:t})})]})})};q.displayName="Viewer";var F=q},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[308,689,857,971,23,744],function(){return e(e.s=1338)}),_N_E=e.O()}]);