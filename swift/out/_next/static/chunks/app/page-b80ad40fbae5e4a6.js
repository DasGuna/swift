(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{8307:function(e,t,a){Promise.resolve().then(a.bind(a,2480)),Promise.resolve().then(a.t.bind(a,2506,23))},2480:function(e,t,a){"use strict";a.d(t,{default:function(){return $}});var o=a(7437),s=a(9449),r=a.n(s),n=a(7776),l=a(7585),i=a(2265),c=a(6547),d=a(6862),u=a(7954);let f=e=>{let{viewport:t,set:a}=(0,d.D)(),{width:s,height:r}=t,n=(0,i.useRef)(null);return(0,o.jsx)(u.c,{makeDefault:!0,ref:n,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:r/s})},b=()=>{let{scene:e}=(0,d.D)();return e.background=new n.Color(7895160),e.fog=new n.Fog(.787878,50,60),(0,o.jsxs)("mesh",{receiveShadow:!0,children:[(0,o.jsx)("planeGeometry",{args:[200,200]}),(0,o.jsx)("meshPhongMaterial",{color:4934475,specular:new n.Color(1052688)})]})},h=e=>{let t=(0,i.useRef)(null);return(0,o.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var m=a(6080),j=a(2325),p=a(2618),g=a(1957),x=a(7413),v=a(2851);let y=(e,t,a)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),a&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{y(e,t,a)})},w=e=>{let t=(0,d.H)(x.j,e.url),a=(0,i.useMemo)(()=>t.clone(),[t]);return(0,o.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,o.jsx)("primitive",{object:a,attach:"geometry"}),(0,o.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},E=e=>(0,o.jsx)(i.Fragment,{children:(0,o.jsx)(g.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),C=e=>{let t=(0,d.H)(v.G,e.url),a=(0,i.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,i.useEffect)(()=>{a.children.forEach(t=>{y(t,e.opacity,!0)}),a.name="loaded"},[a,e.opacity]),(0,o.jsx)("primitive",{object:a,position:e.t,scale:e.scale,quaternion:e.q})};var S=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,o.jsx)(o.Fragment,{});{let t=e.filename.split(".").pop(),a=e.filename;switch(a="/retrieve/"+a,null==t?void 0:t.toLowerCase()){case"stl":return(0,o.jsx)(w,{url:a,...e});case"splat":return(0,o.jsx)(E,{url:a,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,o.jsx)(E,{url:a,...e});default:return(0,o.jsx)(C,{url:a,...e})}}};let O=e=>{let[t,a]=(0,i.useState)(!1);return(0,i.useEffect)(()=>{a(!0)},[]),(0,o.jsx)(i.Fragment,{children:t&&(0,o.jsx)(i.Suspense,{fallback:(0,o.jsx)(function(){let{progress:e}=(0,m.S)();return(0,o.jsxs)(j.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,o.jsx)(S,{...e})})})},D=e=>{let t=(0,i.useRef)(null);return(0,o.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,o.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},_=e=>{let t=(0,i.useRef)(null);switch((0,i.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,o.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,o.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,o.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},I=e=>{let t=(0,i.useRef)(null);return(0,i.useEffect)(()=>{t.current&&t.current}),(0,o.jsx)(p.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,o.jsxs)("mesh",{children:[(0,o.jsx)(_,{...e}),(0,o.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},T=e=>{if(!1==e.display)return(0,o.jsx)(i.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,o.jsx)(O,{...e});case"axes":return(0,o.jsx)(D,{...e});default:return(0,o.jsx)(I,{...e})}},k=e=>(0,o.jsx)("group",{children:e.meshes.map((e,t)=>(0,o.jsx)(T,{...e},t))}),q=i.forwardRef((e,t)=>(0,o.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,o.jsx)(k,{meshes:e},t))}));q.displayName="GroupCollection";var N=a(9079),L=a(8832),R=a.n(L);a(605);let V=[{id:"a7b44fc1-ccc8-4331-a1fc-736aee86a26f",data:{label:"EveryN"},position:{x:0,y:0},is_active:1},{id:"95c9fffc-f380-498c-bfc0-1f9f888522b5",data:{label:"Guard"},position:{x:10,y:20},is_active:1},{id:"de645bdf-fcf3-41c3-b3ad-6695932391de",data:{label:"Periodic"},position:{x:20,y:40},is_active:1},{id:"7d060d85-d29d-4a94-a95e-5d683420379f",data:{label:"Finisher"},position:{x:30,y:60},is_active:1},{id:"4aa51604-a928-4e75-803d-4bcc9faa44fb",data:{label:"Sequence"},position:{x:40,y:80},is_active:1},{id:"7fac6682-5b5b-45a9-a69f-1428075189e2",data:{label:"Idle"},position:{x:50,y:100},is_active:0},{id:"60496a73-2c42-4ba8-8f2a-ccf0e7cb5068",data:{label:"Logging"},position:{x:60,y:120},is_active:1}],F=[{id:"4aa51604-a928-4e75-803d-4bcc9faa44fb95c9fffc-f380-498c-bfc0-1f9f888522b5",source:"4aa51604-a928-4e75-803d-4bcc9faa44fb",target:"95c9fffc-f380-498c-bfc0-1f9f888522b5"},{id:"4aa51604-a928-4e75-803d-4bcc9faa44fbde645bdf-fcf3-41c3-b3ad-6695932391de",source:"4aa51604-a928-4e75-803d-4bcc9faa44fb",target:"de645bdf-fcf3-41c3-b3ad-6695932391de"},{id:"4aa51604-a928-4e75-803d-4bcc9faa44fb7d060d85-d29d-4a94-a95e-5d683420379f",source:"4aa51604-a928-4e75-803d-4bcc9faa44fb",target:"7d060d85-d29d-4a94-a95e-5d683420379f"},{id:"60496a73-2c42-4ba8-8f2a-ccf0e7cb5068a7b44fc1-ccc8-4331-a1fc-736aee86a26f",source:"60496a73-2c42-4ba8-8f2a-ccf0e7cb5068",target:"a7b44fc1-ccc8-4331-a1fc-736aee86a26f"},{id:"60496a73-2c42-4ba8-8f2a-ccf0e7cb50684aa51604-a928-4e75-803d-4bcc9faa44fb",source:"60496a73-2c42-4ba8-8f2a-ccf0e7cb5068",target:"4aa51604-a928-4e75-803d-4bcc9faa44fb"},{id:"60496a73-2c42-4ba8-8f2a-ccf0e7cb50687fac6682-5b5b-45a9-a69f-1428075189e2",source:"60496a73-2c42-4ba8-8f2a-ccf0e7cb5068",target:"7fac6682-5b5b-45a9-a69f-1428075189e2"}],G=(e,t,a)=>{let o=new(R()).graphlib.Graph().setDefaultEdgeLabel(()=>({}));return o.setGraph({rankdir:a.direction}),t.forEach(e=>o.setEdge(e.source,e.target)),e.forEach(e=>{var t,a,s,r;return o.setNode(e.id,{...e,width:null!==(s=null===(t=e.measured)||void 0===t?void 0:t.width)&&void 0!==s?s:0,height:null!==(r=null===(a=e.measured)||void 0===a?void 0:a.height)&&void 0!==r?r:0})}),R().layout(o),{nodes:e.map(e=>{var t,a,s,r;let n=o.node(e.id),l=n.x-(null!==(s=null===(t=e.measured)||void 0===t?void 0:t.width)&&void 0!==s?s:0)/2,i=n.y-(null!==(r=null===(a=e.measured)||void 0===a?void 0:a.height)&&void 0!==r?r:0)/2;return{...e,position:{x:l,y:i}}}),edges:t}},B=()=>{let{fitView:e}=(0,N._K)(),[t,a,s]=(0,N.Rr)(V),[r,n,l]=(0,N.ll)(F),c=(0,i.useCallback)(o=>{console.log(t);let s=G(t,r,{direction:o});a([...s.nodes]),n([...s.edges]),window.requestAnimationFrame(()=>{e()})},[t,r]);return(0,o.jsx)("div",{style:{height:400},children:(0,o.jsx)(N.x$,{nodes:t,edges:r,onNodesChange:s,onEdgesChange:l,defaultNodes:V,defaultEdges:F,defaultEdgeOptions:{animated:!0,style:{stroke:"white"}},style:{backgroundColor:"#D3D2E5"},fitView:!0,connectionLineStyle:{stroke:"white"},onInit:()=>c("TB"),children:(0,o.jsxs)(N.s_,{position:"bottom-left",children:[(0,o.jsx)("button",{onClick:()=>c("TB"),children:" Vertical "}),(0,o.jsx)("button",{onClick:()=>c("LR"),children:" Horizontal "})]})})})};function P(){return(0,o.jsx)(N.tV,{children:(0,o.jsx)(B,{})})}let z=new(a(6731)).EventEmitter;var K=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let a={...e.formData},o=[...e.formElements];return a[t.index]=t.data,o[t.index-3][t.valueName]=t.value,{formElements:o,formData:a};case"userInputNoState":let s={...e.formData};return s[t.index]=t.data,{formElements:[...e.formElements],formData:s};case"wsUpdate":let r=[...e.formElements];return r[t.index-3]=t.data,{...e,formElements:r};case"reset":let n={...e.formData};return t.indices.map(e=>{delete n[e]}),{formData:n,formElements:[...e.formElements]};default:throw Error()}},M=a(3407),A=a(2844);let W={objects:[],objectList:{NONE:-1},noObjects:0,compID:0,targetID:0,treeVisible:!1},H=(0,M.Ue)((0,A.n)(e=>({...W,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),toggleTreeVisibility:()=>e(e=>{e.treeVisible=!e.treeVisible}),setTargetID:t=>e(e=>{e.targetID=t}),addObject:(t,a)=>e(e=>{e.noObjects>0?e.objects=[...e.objects,t]:e.objects=[t],e.objectList[a]=e.noObjects,e.noObjects+=1}),removeObject:t=>e(e=>{if(e.objects&&e.objects.length&&e.noObjects>0&&t>=0){let a=Object.keys(e.objectList).find(a=>e.objectList[a]===Number(t));console.log("Object key is "+a);let{[a]:o,...s}=e.objectList;Object.keys(s).forEach(function(e){s[e]>t&&(console.log("Value "+s[e]+" is greater than removed "+t),s[e]-=1)}),e.objectList=s,e.noObjects-=1,1===e.objects.length?e.objects=[]:e.objects.splice(t,1)}else console.log("Undefined - Cannot Remove"),alert("Cannot Remove Object -> Object is Invalid or Empty")}),setPos:t=>e(e=>{console.log("targetID: "+e.targetID),e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{if(e.objects[e.targetID]){var a=new n.Euler(t.x,t.y,t.z),o=new n.Quaternion().setFromEuler(a);e.objects[e.targetID][e.compID].euler=[t.x,t.y,t.z],e.objects[e.targetID][e.compID].q=[o.x,o.y,o.z,o.w]}}),setColour:t=>e(e=>{e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].color=t)})})));var U=a(8592),Q=function(){return(0,o.jsx)(U.G,{})};function X(e){let{isOpen:t,onClose:a,children:s,setCloseButton:r}=e;return(0,o.jsx)(o.Fragment,{children:t?(0,o.jsxs)("div",{className:"overlay",children:[(0,o.jsx)("div",{className:"overlay_background",onClick:a}),(0,o.jsxs)("div",{className:"overlay_container",children:[r?(0,o.jsx)("div",{className:"overlay_controls",children:(0,o.jsx)("button",{className:"overlay_close_button",type:"button",onClick:a})}):null,s]})]}):null})}a(1472);var Y=a(8154),J=function(){let e=H(e=>e.objects),t=H(e=>e.targetID),a=H(e=>e.compID),s=H(e=>e.noObjects),r=H(e=>e.setObjectVisibility),n=H(e=>e.setTargetID),l=H(e=>e.setPos),c=H(e=>e.setRot),d=H(e=>e.addObject),u=H(e=>e.removeObject),f=H(e=>e.setColour),b=H(e=>e.objectList),h=H(e=>e.toggleTreeVisibility),m=H(e=>e.treeVisible),[j,p]=(0,i.useState)("cylinder"),[g,x]=(0,i.useState)("#f00"),[v,y]=(0,i.useState)("0"),[w,E]=(0,Y.M4)("Object Handler",()=>({ID:{label:"ID:",options:b,onChange:e=>{n(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>r(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>l(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>c(e)},Colour:{value:"#f00",onChange:e=>f(e)},NoObjects:{label:"Total:",value:s},"Remove Object":(0,Y.LI)(()=>{u(t)})}),[t,s]);return(0,i.useEffect)(()=>{E({NoObjects:s}),e[t]&&(E({Visible:e[t][a].display}),E({Position:e[t][a].t}),E({Rotation:e[t][a].euler}),E({Colour:e[t][a].color}))},[s,t,e,E,a]),(0,Y.M4)("Object Creator",()=>({Type:{options:["cylinder","sphere","cube"],onChange:e=>{p(e)}},Colour:{value:"#0f0",onChange:e=>{x(e)}},Key:{value:"obj_001",onChange:e=>{y(e)}},"Add Object":(0,Y.LI)(()=>{v in b?alert("Cannot Create Object -> Key Already in Use"):d([{stype:j,scale:[1,1,1],filename:void 0,radius:.5,length:1,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:g,opacity:0,display:!0,head_length:0,head_radius:0}],v)})}),[s,j,v]),(0,Y.M4)("Behaviour Tree Handler",()=>({[m?"Close Tree Viewer":"Open Tree Viewer"]:(0,Y.LI)(()=>{h()})}),[m]),(0,o.jsx)(o.Fragment,{})};let Z=e=>{let t=(0,i.useRef)(null),a=(0,i.useRef)(),[s,d]=(0,i.useState)(!1),[u,m]=(0,i.useState)(0),[j,p]=(0,i.useState)(!1),[g,x]=(0,i.useState)(!1),[v,y]=(0,i.useReducer)(K,{formData:{},formElements:[]}),w=H(e=>e.addObject),E=H(e=>e.removeObject),C=H(e=>e.objects),S=H(e=>e.objectList),O=H(e=>e.treeVisible);return(0,i.useEffect)(()=>{let t=!0;x(!0);let o=e.port,s=window.location.search.substring(1).split("&");if(0===o&&(console.log("WEBSOCKET: port is 0, using server_params..."),o=parseInt(s[0])),null===o||isNaN(o))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+o+"/";a.current=new WebSocket(e),t&&(a.current.onopen=()=>{a.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},a.current.send("Connected"),p(!0)},z.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),a.current.send(e)})),a&&(a.current.onmessage=e=>{let t=JSON.parse(e.data),a=t[0],o=t[1];z.emit("wsRx",a,o)})}},[e.port]),(0,i.useEffect)(()=>{let e={shape:e=>{let t=C.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),e&&e[0].uuid?w(e,e[0].uuid):w(e,t),z.emit("wsSwiftTx",t)},shape_mounted:e=>{let a=e[0],o=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+a);try{var s;let e=0;null===(s=t.current)||void 0===s||s.children[a].children.forEach((t,a)=>{"loaded"===t.name&&e++}),e===o?z.emit("wsSwiftTx","1"):z.emit("wsSwiftTx","0")}catch(e){console.log(e),z.emit("wsSwiftTx","0")}},remove:e=>{console.log("WEBSOCKET: removing id "+e),E(e),z.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(v.formData).length?(z.emit("wsSwiftTx",JSON.stringify(v.formData)),y({type:"reset",indices:Object.keys(v.formData)})):z.emit("wsSwiftTx","[]"),e.forEach(e=>{let a=e[0],o=e[1],s=S[a];o.forEach((e,a)=>{var o,r,l;if(null===(o=t.current)||void 0===o?void 0:o.children[s].children[a]){null===(r=t.current)||void 0===r||r.children[s].children[a].position.set(e.t[0],e.t[1],e.t[2]);let o=new n.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(l=t.current)||void 0===l||l.children[s].children[a].setRotationFromQuaternion(o)}})})},sim_time:e=>{m(parseFloat(e))},close:()=>{a.current.close()}};z.removeAllListeners("wsRx"),z.on("wsRx",(t,a)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](a)})},[v,w,E,C]),n.Object3D.DEFAULT_UP=new n.Vector3(0,0,1),(0,o.jsxs)("div",{className:r().visContainer,children:[(0,o.jsx)(X,{isOpen:O,children:(0,o.jsx)(P,{})}),(0,o.jsxs)(l.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,o.jsx)(J,{}),(0,o.jsx)(Q,{}),(0,o.jsx)(f,{t:[1,1,1]}),(0,o.jsx)("hemisphereLight",{groundColor:new n.Color(1118498)}),(0,o.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(c.z,{panSpeed:.5,rotateSpeed:.4}),(0,o.jsx)("axesHelper",{args:[100]}),(0,o.jsx)(b,{}),(0,o.jsx)(i.Suspense,{fallback:null,children:(0,o.jsx)(q,{meshes:C,ref:t})})]})]})};Z.displayName="Viewer";var $=Z},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},1472:function(e){e.exports={overlay_background:"overlay_overlay_background__qcqTT",overlay_container:"overlay_overlay_container__qRGGS"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[471,689,319,776,971,23,744],function(){return e(e.s=8307)}),_N_E=e.O()}]);