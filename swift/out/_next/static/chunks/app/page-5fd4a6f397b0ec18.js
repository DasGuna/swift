(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{9327:function(e,t,s){Promise.resolve().then(s.bind(s,883)),Promise.resolve().then(s.bind(s,6849)),Promise.resolve().then(s.t.bind(s,2506,23))},883:function(e,t,s){"use strict";var r=s(7437),o=s(2265),n=s(8154),a=s(2888);t.default=function(){let e=(0,a.o)(e=>e.objects),t=(0,a.o)(e=>e.targetID),s=(0,a.o)(e=>e.compID),l=(0,a.o)(e=>e.noObjects),i=(0,a.o)(e=>e.setObjectVisibility),c=(0,a.o)(e=>e.setTargetID),u=(0,a.o)(e=>e.setPos),d=(0,a.o)(e=>e.setRot),m=(0,a.o)(e=>e.addObject),f=(0,a.o)(e=>e.idList),[h,p]=(0,n.M4)("Object",()=>({ID:{label:"Object ID:",options:f,onChange:e=>{c(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>i(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>u(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>d(e)},NoObjects:{label:"Total:",value:l}}),[f]);return(0,o.useEffect)(()=>{p({NoObjects:l}),e[t]&&(p({Visible:e[t][s].display}),p({Position:e[t][s].t}))},[l,t,e,p,s]),(0,n.M4)("Object Tester",()=>({Add:(0,n.LI)(()=>{m([{stype:"cylinder",scale:[1,1,1],filename:void 0,radius:.4,length:1,euler:[0,0,0],q:[0,0,0,1],t:[-1,0,0],v:[0,0,0],color:"green",opacity:0,display:!0,head_length:0,head_radius:0}],l)})}),[l]),(0,r.jsx)(r.Fragment,{})}},2888:function(e,t,s){"use strict";s.d(t,{o:function(){return a}});var r=s(9099),o=s(2844);let n={objects:[],noObjects:0,compID:0,targetID:0,idList:[0]},a=(0,r.Ue)((0,o.n)(e=>({...n,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),setTargetID:t=>e(e=>{t===e.idList||e.idList?e.targetID=t:(console.log(t),e.targetID=e.targetID,alert("Invalid Target ID"))}),addObject:(t,s)=>e(e=>{e.noObjects>0?(e.objects=[...e.objects,t],e.idList=[...e.idList,s]):(e.objects=[t],e.idList=[s]),e.noObjects+=1}),setPos:t=>e(e=>{e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{e.objects[e.targetID]&&console.log("SET ROTATION: IN DEV")})})))},6849:function(e,t,s){"use strict";s.d(t,{default:function(){return M}});var r=s(7437),o=s(7776),n=s(3149),a=s(2265),l=s(6547),i=s(9449),c=s.n(i),u=s(9367),d=s(5533);let m=e=>{let{viewport:t,set:s}=(0,u.A)(),{width:o,height:n}=t,l=(0,a.useRef)(null);return(0,r.jsx)(d.c,{makeDefault:!0,ref:l,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/o})},f=()=>{let{scene:e}=(0,u.A)();return e.background=new o.Color(7895160),e.fog=new o.Fog(.787878,50,60),(0,r.jsxs)("mesh",{receiveShadow:!0,children:[(0,r.jsx)("planeGeometry",{args:[200,200]}),(0,r.jsx)("meshPhongMaterial",{color:4934475,specular:new o.Color(1052688)})]})},h=e=>{let t=(0,a.useRef)(null);return(0,r.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var p=s(7581),j=s(2325),x=s(2618),g=s(1957),E=s(7413),b=s(2851);let w=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{w(e,t,s)})},y=e=>{let t=(0,u.F)(E.j,e.url),s=(0,a.useMemo)(()=>t.clone(),[t]);return(0,r.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,r.jsx)("primitive",{object:s,attach:"geometry"}),(0,r.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},S=e=>(0,r.jsx)(a.Fragment,{children:(0,r.jsx)(g.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),v=e=>{let t=(0,u.F)(b.G,e.url),s=(0,a.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,a.useEffect)(()=>{s.children.forEach(t=>{w(t,e.opacity,!0)}),s.name="loaded"},[s,e.opacity]),(0,r.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var D=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,r.jsx)(r.Fragment,{});{let t=e.filename.split(".").pop(),s=e.filename;switch(s="/retrieve/"+s,null==t?void 0:t.toLowerCase()){case"stl":return(0,r.jsx)(y,{url:s,...e});case"splat":return(0,r.jsx)(S,{url:s,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,r.jsx)(S,{url:s,...e});default:return(0,r.jsx)(v,{url:s,...e})}}};let O=e=>{let[t,s]=(0,a.useState)(!1);return(0,a.useEffect)(()=>{s(!0)},[]),(0,r.jsx)(a.Fragment,{children:t&&(0,r.jsx)(a.Suspense,{fallback:(0,r.jsx)(function(){let{progress:e}=(0,p.S)();return(0,r.jsxs)(j.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,r.jsx)(D,{...e})})})},I=e=>{let t=(0,a.useRef)(null);return(0,r.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,r.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},T=e=>{let t=(0,a.useRef)(null);switch((0,a.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,r.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,r.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,r.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},C=e=>{let t=(0,a.useRef)(null);return(0,a.useEffect)(()=>{t.current&&t.current}),(0,r.jsx)(x.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,r.jsxs)("mesh",{children:[(0,r.jsx)(T,{...e}),(0,r.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},q=e=>{if(!1==e.display)return(0,r.jsx)(a.Fragment,{});switch(e.stype){case"mesh":return console.log("SHAPE: MESH"),(0,r.jsx)(O,{...e});case"splat":return console.log("SHAPE: SPLAT"),(0,r.jsx)(O,{...e});case"axes":return console.log("SHAPE: AXES"),(0,r.jsx)(I,{...e});default:return console.log("SHAPE: DEFAULT"),(0,r.jsx)(C,{...e})}},_=e=>(0,r.jsx)("group",{children:e.meshes.map((e,t)=>(0,r.jsx)(q,{...e},t))}),L=a.forwardRef((e,t)=>(0,r.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,r.jsx)(_,{meshes:e},t))}));L.displayName="GroupCollection";var R=s(8592),A=function(){return(0,r.jsx)(R.G,{})};let N=new(s(6731)).EventEmitter;var P=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},r=[...e.formElements];return s[t.index]=t.data,r[t.index-3][t.valueName]=t.value,{formElements:r,formData:s};case"userInputNoState":let o={...e.formData};return o[t.index]=t.data,{formElements:[...e.formElements],formData:o};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let a={...e.formData};return t.indices.map(e=>{delete a[e]}),{formData:a,formElements:[...e.formElements]};default:throw Error()}},k=s(2888);let F=e=>{let t=(0,a.useRef)(null),s=(0,a.useRef)(),[i,u]=(0,a.useState)(0),[d,p]=(0,a.useState)(!1),[j,x]=(0,a.useState)(!1),[g,E]=(0,a.useReducer)(P,{formData:{},formElements:[]}),b=(0,k.o)(e=>e.addObject),w=(0,k.o)(e=>e.objects);return(0,a.useEffect)(()=>{let t=!0;x(!0);let r=e.port,o=window.location.search.substring(1).split("&");if(0===r&&(console.log("WEBSOCKET: port is 0, using server_params..."),r=parseInt(o[0])),null===r||isNaN(r))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+r+"/";s.current=new WebSocket(e),t&&(s.current.onopen=()=>{s.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},s.current.send("Connected"),p(!0)},N.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),s.current.send(e)})),s&&(s.current.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],r=t[1];N.emit("wsRx",s,r)})}},[e.port]),(0,a.useEffect)(()=>{let e={shape:e=>{let t=w.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),b(e,t),N.emit("wsSwiftTx",t)},shape_mounted:e=>{let s=e[0],r=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+s);try{var o;let e=0;null===(o=t.current)||void 0===o||o.children[s].children.forEach((t,s)=>{"loaded"===t.name&&e++}),e===r?N.emit("wsSwiftTx","1"):N.emit("wsSwiftTx","0")}catch(e){console.log(e),N.emit("wsSwiftTx","0")}},remove:e=>{N.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(g.formData).length?(N.emit("wsSwiftTx",JSON.stringify(g.formData)),E({type:"reset",indices:Object.keys(g.formData)})):N.emit("wsSwiftTx","[]"),e.forEach(e=>{let s=e[0];e[1].forEach((e,r)=>{var n,a,l;if(null===(n=t.current)||void 0===n?void 0:n.children[s].children[r]){null===(a=t.current)||void 0===a||a.children[s].children[r].position.set(e.t[0],e.t[1],e.t[2]);let n=new o.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(l=t.current)||void 0===l||l.children[s].children[r].setRotationFromQuaternion(n)}})})},sim_time:e=>{u(parseFloat(e))},close:e=>{s.current.close()}};N.removeAllListeners("wsRx"),N.on("wsRx",(t,s)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](s)})},[g,b,w]),o.Object3D.DEFAULT_UP=new o.Vector3(0,0,1),(0,r.jsx)("div",{className:c().visContainer,children:(0,r.jsxs)(n.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,r.jsx)(A,{}),(0,r.jsx)(m,{t:[1,1,1]}),(0,r.jsx)("hemisphereLight",{groundColor:new o.Color(1118498)}),(0,r.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(l.z,{panSpeed:.5,rotateSpeed:.4}),(0,r.jsx)("axesHelper",{args:[100]}),(0,r.jsx)(f,{}),(0,r.jsx)(a.Suspense,{fallback:null,children:(0,r.jsx)(L,{meshes:w,ref:t})})]})})};F.displayName="Viewer";var M=F},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[308,689,954,971,23,744],function(){return e(e.s=9327)}),_N_E=e.O()}]);