(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{5129:function(e,t,s){Promise.resolve().then(s.bind(s,883)),Promise.resolve().then(s.bind(s,3867)),Promise.resolve().then(s.t.bind(s,2506,23))},883:function(e,t,s){"use strict";var o=s(7437),r=s(2265),n=s(8154),l=s(2888);t.default=function(){let e=(0,l.o)(e=>e.objects),t=(0,l.o)(e=>e.targetID),s=(0,l.o)(e=>e.compID),a=(0,l.o)(e=>e.noObjects),i=(0,l.o)(e=>e.setObjectVisibility),c=(0,l.o)(e=>e.setTargetID),u=(0,l.o)(e=>e.setPos),d=(0,l.o)(e=>e.setRot),m=(0,l.o)(e=>e.addObject),f=(0,l.o)(e=>e.idList),h=(0,l.o)(e=>e.toggleTreeVisibility),p=(0,l.o)(e=>e.treeVisible),[j,g]=(0,n.M4)("Object",()=>({ID:{label:"Object ID:",options:f,onChange:e=>{c(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>i(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>u(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>d(e)},NoObjects:{label:"Total:",value:a}}),[f]);return(0,r.useEffect)(()=>{g({NoObjects:a}),e[t]&&(g({Visible:e[t][s].display}),g({Position:e[t][s].t}))},[a,t,e,g,s]),(0,n.M4)("Object Tester",()=>({"Add Cylinder":(0,n.LI)(()=>{m([{stype:"cylinder",scale:[1,1,1],filename:void 0,radius:.4,length:1,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:"green",opacity:0,display:!0,head_length:0,head_radius:0}],a)})}),[a]),(0,n.M4)("Behaviour Tree Handler",()=>({[p?"Close Tree Viewer":"Open Tree Viewer"]:(0,n.LI)(()=>{h()})}),[p]),(0,o.jsx)(o.Fragment,{})}},2888:function(e,t,s){"use strict";s.d(t,{o:function(){return l}});var o=s(3407),r=s(2844);let n={objects:[],noObjects:0,compID:0,targetID:0,idList:[0],treeVisible:!1},l=(0,o.Ue)((0,r.n)(e=>({...n,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),toggleTreeVisibility:()=>e(e=>{e.treeVisible=!e.treeVisible,console.log("Toggling tree visibility"),console.log(e.treeVisible)}),setTargetID:t=>e(e=>{t===e.idList||e.idList?e.targetID=t:(console.log(t),e.targetID=e.targetID,alert("Invalid Target ID"))}),addObject:(t,s)=>e(e=>{e.noObjects>0?(e.objects=[...e.objects,t],e.idList=[...e.idList,s]):(e.objects=[t],e.idList=[s]),e.noObjects+=1}),removeObject:t=>e(e=>{var s=e.idList.indexOf(t);console.log("ID from list is "+s),console.log("Deleting object "+e.objects[t]),-1!==t&&e.objects.splice(t,1),-1!==s&&e.idList.splice(s,1),e.noObjects-=1}),setPos:t=>e(e=>{e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{e.objects[e.targetID]&&(console.log("SET ROTATION: IN DEV"),e.objects[e.targetID][e.compID].q=[t.w,t.x,t.y,t.z])})})))},3867:function(e,t,s){"use strict";s.d(t,{default:function(){return W}});var o=s(7437),r=s(7776),n=s(7585),l=s(2265),a=s(6547),i=s(9449),c=s.n(i),u=s(6862),d=s(7954);let m=e=>{let{viewport:t,set:s}=(0,u.D)(),{width:r,height:n}=t,a=(0,l.useRef)(null);return(0,o.jsx)(d.c,{makeDefault:!0,ref:a,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/r})},f=()=>{let{scene:e}=(0,u.D)();return e.background=new r.Color(7895160),e.fog=new r.Fog(.787878,50,60),(0,o.jsxs)("mesh",{receiveShadow:!0,children:[(0,o.jsx)("planeGeometry",{args:[200,200]}),(0,o.jsx)("meshPhongMaterial",{color:4934475,specular:new r.Color(1052688)})]})},h=e=>{let t=(0,l.useRef)(null);return(0,o.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var p=s(6080),j=s(2325),g=s(2618),x=s(1957),b=s(7413),y=s(2851);let v=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{v(e,t,s)})},w=e=>{let t=(0,u.H)(b.j,e.url),s=(0,l.useMemo)(()=>t.clone(),[t]);return(0,o.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,o.jsx)("primitive",{object:s,attach:"geometry"}),(0,o.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},E=e=>(0,o.jsx)(l.Fragment,{children:(0,o.jsx)(x.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),S=e=>{let t=(0,u.H)(y.G,e.url),s=(0,l.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,l.useEffect)(()=>{s.children.forEach(t=>{v(t,e.opacity,!0)}),s.name="loaded"},[s,e.opacity]),(0,o.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var D=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,o.jsx)(o.Fragment,{});{let t=e.filename.split(".").pop(),s=e.filename;switch(s="/retrieve/"+s,null==t?void 0:t.toLowerCase()){case"stl":return(0,o.jsx)(w,{url:s,...e});case"splat":return(0,o.jsx)(E,{url:s,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,o.jsx)(E,{url:s,...e});default:return(0,o.jsx)(S,{url:s,...e})}}};let O=e=>{let[t,s]=(0,l.useState)(!1);return(0,l.useEffect)(()=>{s(!0)},[]),(0,o.jsx)(l.Fragment,{children:t&&(0,o.jsx)(l.Suspense,{fallback:(0,o.jsx)(function(){let{progress:e}=(0,p.S)();return(0,o.jsxs)(j.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,o.jsx)(D,{...e})})})},C=e=>{let t=(0,l.useRef)(null);return(0,o.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,o.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},I=e=>{let t=(0,l.useRef)(null);switch((0,l.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,o.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,o.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,o.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},T=e=>{let t=(0,l.useRef)(null);return(0,l.useEffect)(()=>{t.current&&t.current}),(0,o.jsx)(g.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,o.jsxs)("mesh",{children:[(0,o.jsx)(I,{...e}),(0,o.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},q=e=>{if(!1==e.display)return(0,o.jsx)(l.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,o.jsx)(O,{...e});case"axes":return(0,o.jsx)(C,{...e});default:return(0,o.jsx)(T,{...e})}},N=e=>(0,o.jsx)("group",{children:e.meshes.map((e,t)=>(0,o.jsx)(q,{...e},t))}),_=l.forwardRef((e,t)=>(0,o.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,o.jsx)(N,{meshes:e},t))}));_.displayName="GroupCollection";var k=s(8592),L=function(){return(0,o.jsx)(k.G,{})},R=s(9079);function V(){return(0,o.jsx)(R.x$,{defaultNodes:[{id:"a",type:"input",data:{label:"Node A"},position:{x:200,y:10}},{id:"b",type:"input",data:{label:"Node B"},position:{x:100,y:100}}],defaultEdges:[{id:"ea-b",source:"a",target:"b"}],defaultEdgeOptions:{animated:!0,style:{stroke:"white"}},fitView:!0,style:{backgroundColor:"#D3D2E5"},connectionLineStyle:{stroke:"white"}})}s(605);let B=new(s(6731)).EventEmitter;var P=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},o=[...e.formElements];return s[t.index]=t.data,o[t.index-3][t.valueName]=t.value,{formElements:o,formData:s};case"userInputNoState":let r={...e.formData};return r[t.index]=t.data,{formElements:[...e.formElements],formData:r};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let l={...e.formData};return t.indices.map(e=>{delete l[e]}),{formData:l,formElements:[...e.formElements]};default:throw Error()}},M=s(2888);function A(e){let{isOpen:t,onClose:s,children:r,setCloseButton:n}=e;return(0,o.jsx)(o.Fragment,{children:t?(0,o.jsxs)("div",{className:"overlay",children:[(0,o.jsx)("div",{className:"overlay_background",onClick:s}),(0,o.jsxs)("div",{className:"overlay_container",children:[n?(0,o.jsx)("div",{className:"overlay_controls",children:(0,o.jsx)("button",{className:"overlay_close_button",type:"button",onClick:s})}):null,r]})]}):null})}s(1472);let F=e=>{let t=(0,l.useRef)(null),s=(0,l.useRef)(),[i,u]=(0,l.useState)(!1),[d,p]=(0,l.useState)(0),[j,g]=(0,l.useState)(!1),[x,b]=(0,l.useState)(!1),[y,v]=(0,l.useReducer)(P,{formData:{},formElements:[]}),w=(0,M.o)(e=>e.addObject),E=(0,M.o)(e=>e.removeObject),S=(0,M.o)(e=>e.objects),D=(0,M.o)(e=>e.treeVisible);return(0,l.useEffect)(()=>{let t=!0;b(!0);let o=e.port,r=window.location.search.substring(1).split("&");if(0===o&&(console.log("WEBSOCKET: port is 0, using server_params..."),o=parseInt(r[0])),null===o||isNaN(o))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+o+"/";s.current=new WebSocket(e),t&&(s.current.onopen=()=>{s.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},s.current.send("Connected"),g(!0)},B.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),s.current.send(e)})),s&&(s.current.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],o=t[1];B.emit("wsRx",s,o)})}},[e.port]),(0,l.useEffect)(()=>{let e={shape:e=>{let t=S.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),w(e,t),B.emit("wsSwiftTx",t)},shape_mounted:e=>{let s=e[0],o=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+s);try{var r;let e=0;null===(r=t.current)||void 0===r||r.children[s].children.forEach((t,s)=>{"loaded"===t.name&&e++}),e===o?B.emit("wsSwiftTx","1"):B.emit("wsSwiftTx","0")}catch(e){console.log(e),B.emit("wsSwiftTx","0")}},remove:e=>{console.log("WEBSOCKET: removing id "+e),E(e),B.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(y.formData).length?(B.emit("wsSwiftTx",JSON.stringify(y.formData)),v({type:"reset",indices:Object.keys(y.formData)})):B.emit("wsSwiftTx","[]"),e.forEach(e=>{let s=e[0];e[1].forEach((e,o)=>{var n,l,a;if(null===(n=t.current)||void 0===n?void 0:n.children[s].children[o]){null===(l=t.current)||void 0===l||l.children[s].children[o].position.set(e.t[0],e.t[1],e.t[2]);let n=new r.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(a=t.current)||void 0===a||a.children[s].children[o].setRotationFromQuaternion(n)}})})},sim_time:e=>{p(parseFloat(e))},close:()=>{s.current.close()}};B.removeAllListeners("wsRx"),B.on("wsRx",(t,s)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](s)})},[y,w,E,S]),r.Object3D.DEFAULT_UP=new r.Vector3(0,0,1),(0,o.jsxs)("div",{className:c().visContainer,children:[(0,o.jsx)(A,{isOpen:D,onClose:()=>u(!D),setCloseButton:!0,children:(0,o.jsx)(V,{})}),(0,o.jsxs)(n.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,o.jsx)(L,{}),(0,o.jsx)(m,{t:[1,1,1]}),(0,o.jsx)("hemisphereLight",{groundColor:new r.Color(1118498)}),(0,o.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(a.z,{panSpeed:.5,rotateSpeed:.4}),(0,o.jsx)("axesHelper",{args:[100]}),(0,o.jsx)(f,{}),(0,o.jsx)(l.Suspense,{fallback:null,children:(0,o.jsx)(_,{meshes:S,ref:t})})]})]})};F.displayName="Viewer";var W=F},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},1472:function(){},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[471,689,319,875,971,23,744],function(){return e(e.s=5129)}),_N_E=e.O()}]);