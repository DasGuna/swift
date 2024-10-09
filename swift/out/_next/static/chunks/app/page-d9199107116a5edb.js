(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{5129:function(e,t,s){Promise.resolve().then(s.bind(s,883)),Promise.resolve().then(s.bind(s,3867)),Promise.resolve().then(s.t.bind(s,2506,23))},883:function(e,t,s){"use strict";var r=s(7437),o=s(2265),n=s(8154),l=s(2888);t.default=function(){let e=(0,l.o)(e=>e.objects),t=(0,l.o)(e=>e.targetID),s=(0,l.o)(e=>e.compID),a=(0,l.o)(e=>e.noObjects),i=(0,l.o)(e=>e.setObjectVisibility),c=(0,l.o)(e=>e.setTargetID),u=(0,l.o)(e=>e.setPos),d=(0,l.o)(e=>e.setRot),m=(0,l.o)(e=>e.addObject),f=(0,l.o)(e=>e.idList),h=(0,l.o)(e=>e.toggleTreeVisibility),p=(0,l.o)(e=>e.treeVisible);(0,l.o)(e=>e.getRotation);let[j,g]=(0,o.useState)("cylinder"),[x,b]=(0,n.M4)("Object",()=>({ID:{label:"Object ID:",options:f,onChange:e=>{c(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>i(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>u(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>d(e)},NoObjects:{label:"Total:",value:a}}),[f]);return(0,o.useEffect)(()=>{b({NoObjects:a}),e[t]&&(b({Visible:e[t][s].display}),b({Position:e[t][s].t}),b({Rotation:e[t][s].euler}))},[a,t,e,b,s]),(0,n.M4)("Object Handler",()=>({Type:{options:["cylinder","sphere","cube"],onChange:e=>{g(e)}},Colour:{value:"#f00"},"Add Object":(0,n.LI)(()=>{m([{stype:j,scale:[1,1,1],filename:void 0,radius:.5,length:1,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:"green",opacity:0,display:!0,head_length:0,head_radius:0}],a)})}),[a,j]),(0,n.M4)("Behaviour Tree Handler",()=>({[p?"Close Tree Viewer":"Open Tree Viewer"]:(0,n.LI)(()=>{h()})}),[p]),(0,r.jsx)(r.Fragment,{})}},2888:function(e,t,s){"use strict";s.d(t,{o:function(){return a}});var r=s(7776),o=s(3407),n=s(2844);let l={objects:[],noObjects:0,compID:0,targetID:0,idList:[0],treeVisible:!1},a=(0,o.Ue)((0,n.n)(e=>({...l,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),toggleTreeVisibility:()=>e(e=>{e.treeVisible=!e.treeVisible,console.log("Toggling tree visibility"),console.log(e.treeVisible)}),setTargetID:t=>e(e=>{t===e.idList||e.idList?e.targetID=t:(console.log(t),e.targetID=e.targetID,alert("Invalid Target ID"))}),addObject:(t,s)=>e(e=>{e.noObjects>0?(e.objects=[...e.objects,t],e.idList=[...e.idList,s]):(e.objects=[t],e.idList=[s]),e.noObjects+=1}),removeObject:t=>e(e=>{var s=e.idList.indexOf(t);console.log("ID from list is "+s),console.log("Deleting object "+e.objects[t]),-1!==t&&e.objects.splice(t,1),-1!==s&&e.idList.splice(s,1),e.noObjects-=1}),setPos:t=>e(e=>{e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{if(e.objects[e.targetID]){console.log("SET ROTATION: IN DEV");var s=new r.Euler(t.x,t.y,t.z),o=new r.Quaternion().setFromEuler(s);e.objects[e.targetID][e.compID].euler=[t.x,t.y,t.z],e.objects[e.targetID][e.compID].q=[o.x,o.y,o.z,o.w]}})})))},3867:function(e,t,s){"use strict";s.d(t,{default:function(){return B}});var r=s(7437),o=s(7776),n=s(7585),l=s(2265),a=s(6547),i=s(9449),c=s.n(i),u=s(6862),d=s(7954);let m=e=>{let{viewport:t,set:s}=(0,u.D)(),{width:o,height:n}=t,a=(0,l.useRef)(null);return(0,r.jsx)(d.c,{makeDefault:!0,ref:a,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/o})},f=()=>{let{scene:e}=(0,u.D)();return e.background=new o.Color(7895160),e.fog=new o.Fog(.787878,50,60),(0,r.jsxs)("mesh",{receiveShadow:!0,children:[(0,r.jsx)("planeGeometry",{args:[200,200]}),(0,r.jsx)("meshPhongMaterial",{color:4934475,specular:new o.Color(1052688)})]})},h=e=>{let t=(0,l.useRef)(null);return(0,r.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var p=s(6080),j=s(2325),g=s(2618),x=s(1957),b=s(7413),y=s(2851);let v=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{v(e,t,s)})},w=e=>{let t=(0,u.H)(b.j,e.url),s=(0,l.useMemo)(()=>t.clone(),[t]);return(0,r.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,r.jsx)("primitive",{object:s,attach:"geometry"}),(0,r.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},E=e=>(0,r.jsx)(l.Fragment,{children:(0,r.jsx)(x.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),D=e=>{let t=(0,u.H)(y.G,e.url),s=(0,l.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,l.useEffect)(()=>{s.children.forEach(t=>{v(t,e.opacity,!0)}),s.name="loaded"},[s,e.opacity]),(0,r.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var S=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,r.jsx)(r.Fragment,{});{let t=e.filename.split(".").pop(),s=e.filename;switch(s="/retrieve/"+s,null==t?void 0:t.toLowerCase()){case"stl":return(0,r.jsx)(w,{url:s,...e});case"splat":return(0,r.jsx)(E,{url:s,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,r.jsx)(E,{url:s,...e});default:return(0,r.jsx)(D,{url:s,...e})}}};let O=e=>{let[t,s]=(0,l.useState)(!1);return(0,l.useEffect)(()=>{s(!0)},[]),(0,r.jsx)(l.Fragment,{children:t&&(0,r.jsx)(l.Suspense,{fallback:(0,r.jsx)(function(){let{progress:e}=(0,p.S)();return(0,r.jsxs)(j.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,r.jsx)(S,{...e})})})},T=e=>{let t=(0,l.useRef)(null);return(0,r.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,r.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},_=e=>{let t=(0,l.useRef)(null);switch((0,l.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,r.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,r.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,r.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},I=e=>{let t=(0,l.useRef)(null);return(0,l.useEffect)(()=>{t.current&&t.current}),(0,r.jsx)(g.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,r.jsxs)("mesh",{children:[(0,r.jsx)(_,{...e}),(0,r.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},C=e=>{if(!1==e.display)return(0,r.jsx)(l.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,r.jsx)(O,{...e});case"axes":return(0,r.jsx)(T,{...e});default:return(0,r.jsx)(I,{...e})}},q=e=>(0,r.jsx)("group",{children:e.meshes.map((e,t)=>(0,r.jsx)(C,{...e},t))}),N=l.forwardRef((e,t)=>(0,r.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,r.jsx)(q,{meshes:e},t))}));N.displayName="GroupCollection";var k=s(8592),L=function(){return(0,r.jsx)(k.G,{})},R=s(9079);function V(){return(0,r.jsx)("div",{style:{height:300},children:(0,r.jsx)(R.x$,{defaultNodes:[{id:"a",type:"input",data:{label:"NO DATA"},position:{x:200,y:10}}],defaultEdges:[{id:"ea-b",source:"a",target:"b"}],defaultEdgeOptions:{animated:!0,style:{stroke:"white"}},fitView:!0,style:{backgroundColor:"#D3D2E5"},connectionLineStyle:{stroke:"white"}})})}s(605);let P=new(s(6731)).EventEmitter;var A=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},r=[...e.formElements];return s[t.index]=t.data,r[t.index-3][t.valueName]=t.value,{formElements:r,formData:s};case"userInputNoState":let o={...e.formData};return o[t.index]=t.data,{formElements:[...e.formElements],formData:o};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let l={...e.formData};return t.indices.map(e=>{delete l[e]}),{formData:l,formElements:[...e.formElements]};default:throw Error()}},F=s(2888);function M(e){let{isOpen:t,onClose:s,children:o,setCloseButton:n}=e;return(0,r.jsx)(r.Fragment,{children:t?(0,r.jsxs)("div",{className:"overlay",children:[(0,r.jsx)("div",{className:"overlay_background",onClick:s}),(0,r.jsxs)("div",{className:"overlay_container",children:[n?(0,r.jsx)("div",{className:"overlay_controls",children:(0,r.jsx)("button",{className:"overlay_close_button",type:"button",onClick:s})}):null,o]})]}):null})}s(1472);let z=e=>{let t=(0,l.useRef)(null),s=(0,l.useRef)(),[i,u]=(0,l.useState)(!1),[d,p]=(0,l.useState)(0),[j,g]=(0,l.useState)(!1),[x,b]=(0,l.useState)(!1),[y,v]=(0,l.useReducer)(A,{formData:{},formElements:[]}),w=(0,F.o)(e=>e.addObject),E=(0,F.o)(e=>e.removeObject),D=(0,F.o)(e=>e.objects),S=(0,F.o)(e=>e.treeVisible);return(0,l.useEffect)(()=>{let t=!0;b(!0);let r=e.port,o=window.location.search.substring(1).split("&");if(0===r&&(console.log("WEBSOCKET: port is 0, using server_params..."),r=parseInt(o[0])),null===r||isNaN(r))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+r+"/";s.current=new WebSocket(e),t&&(s.current.onopen=()=>{s.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},s.current.send("Connected"),g(!0)},P.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),s.current.send(e)})),s&&(s.current.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],r=t[1];P.emit("wsRx",s,r)})}},[e.port]),(0,l.useEffect)(()=>{let e={shape:e=>{let t=D.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),w(e,t),P.emit("wsSwiftTx",t)},shape_mounted:e=>{let s=e[0],r=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+s);try{var o;let e=0;null===(o=t.current)||void 0===o||o.children[s].children.forEach((t,s)=>{"loaded"===t.name&&e++}),e===r?P.emit("wsSwiftTx","1"):P.emit("wsSwiftTx","0")}catch(e){console.log(e),P.emit("wsSwiftTx","0")}},remove:e=>{console.log("WEBSOCKET: removing id "+e),E(e),P.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(y.formData).length?(P.emit("wsSwiftTx",JSON.stringify(y.formData)),v({type:"reset",indices:Object.keys(y.formData)})):P.emit("wsSwiftTx","[]"),e.forEach(e=>{let s=e[0];e[1].forEach((e,r)=>{var n,l,a;if(null===(n=t.current)||void 0===n?void 0:n.children[s].children[r]){null===(l=t.current)||void 0===l||l.children[s].children[r].position.set(e.t[0],e.t[1],e.t[2]);let n=new o.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(a=t.current)||void 0===a||a.children[s].children[r].setRotationFromQuaternion(n)}})})},sim_time:e=>{p(parseFloat(e))},close:()=>{s.current.close()}};P.removeAllListeners("wsRx"),P.on("wsRx",(t,s)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](s)})},[y,w,E,D]),o.Object3D.DEFAULT_UP=new o.Vector3(0,0,1),(0,r.jsxs)("div",{className:c().visContainer,children:[(0,r.jsx)(M,{isOpen:S,children:(0,r.jsx)(V,{})}),(0,r.jsxs)(n.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,r.jsx)(L,{}),(0,r.jsx)(m,{t:[1,1,1]}),(0,r.jsx)("hemisphereLight",{groundColor:new o.Color(1118498)}),(0,r.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(a.z,{panSpeed:.5,rotateSpeed:.4}),(0,r.jsx)("axesHelper",{args:[100]}),(0,r.jsx)(f,{}),(0,r.jsx)(l.Suspense,{fallback:null,children:(0,r.jsx)(N,{meshes:D,ref:t})})]})]})};z.displayName="Viewer";var B=z},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},1472:function(e){e.exports={overlay_background:"overlay_overlay_background__qcqTT",overlay_container:"overlay_overlay_container__qRGGS"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[471,689,319,875,971,23,744],function(){return e(e.s=5129)}),_N_E=e.O()}]);