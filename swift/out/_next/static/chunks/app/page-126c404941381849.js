(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{8307:function(e,t,o){Promise.resolve().then(o.bind(o,2480)),Promise.resolve().then(o.t.bind(o,2506,23))},2480:function(e,t,o){"use strict";o.d(t,{default:function(){return $}});var r=o(7437),s=o(9449),n=o.n(s),a=o(7776),i=o(7585),l=o(2265),c=o(6547),d=o(6862),u=o(7954);let b=e=>{let{viewport:t,set:o}=(0,d.D)(),{width:s,height:n}=t,a=(0,l.useRef)(null);return(0,r.jsx)(u.c,{makeDefault:!0,ref:a,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/s})},f=()=>{let{scene:e}=(0,d.D)();return e.background=new a.Color(7895160),e.fog=new a.Fog(.787878,50,60),(0,r.jsxs)("mesh",{receiveShadow:!0,children:[(0,r.jsx)("planeGeometry",{args:[200,200]}),(0,r.jsx)("meshPhongMaterial",{color:4934475,specular:new a.Color(1052688)})]})},h=e=>{let t=(0,l.useRef)(null);return(0,r.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var m=o(6080),j=o(2325),g=o(2618),p=o(1957),x=o(7413),v=o(2851);let y=(e,t,o)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),o&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{y(e,t,o)})},w=e=>{let t=(0,d.H)(x.j,e.url),o=(0,l.useMemo)(()=>t.clone(),[t]);return(0,r.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,r.jsx)("primitive",{object:o,attach:"geometry"}),(0,r.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},E=e=>(0,r.jsx)(l.Fragment,{children:(0,r.jsx)(p.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),S=e=>{let t=(0,d.H)(v.G,e.url),o=(0,l.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,l.useEffect)(()=>{o.children.forEach(t=>{y(t,e.opacity,!0)}),o.name="loaded"},[o,e.opacity]),(0,r.jsx)("primitive",{object:o,position:e.t,scale:e.scale,quaternion:e.q})};var _=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,r.jsx)(r.Fragment,{});{let t=e.filename.split(".").pop(),o=e.filename;switch(o="/retrieve/"+o,null==t?void 0:t.toLowerCase()){case"stl":return(0,r.jsx)(w,{url:o,...e});case"splat":return(0,r.jsx)(E,{url:o,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,r.jsx)(E,{url:o,...e});default:return(0,r.jsx)(S,{url:o,...e})}}};let C=e=>{let[t,o]=(0,l.useState)(!1);return(0,l.useEffect)(()=>{o(!0)},[]),(0,r.jsx)(l.Fragment,{children:t&&(0,r.jsx)(l.Suspense,{fallback:(0,r.jsx)(function(){let{progress:e}=(0,m.S)();return(0,r.jsxs)(j.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,r.jsx)(_,{...e})})})},O=e=>{let t=(0,l.useRef)(null);return(0,r.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,r.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},D=e=>{let t=(0,l.useRef)(null);switch((0,l.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,r.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,r.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,r.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},I=e=>{let t=(0,l.useRef)(null);return(0,l.useEffect)(()=>{t.current&&t.current}),(0,r.jsx)(g.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,r.jsxs)("mesh",{children:[(0,r.jsx)(D,{...e}),(0,r.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},q=e=>{if(!1==e.display)return(0,r.jsx)(l.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,r.jsx)(C,{...e});case"axes":return(0,r.jsx)(O,{...e});default:return(0,r.jsx)(I,{...e})}},k=e=>(0,r.jsx)("group",{children:e.meshes.map((e,t)=>(0,r.jsx)(q,{...e},t))}),T=l.forwardRef((e,t)=>(0,r.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,r.jsx)(k,{meshes:e},t))}));T.displayName="GroupCollection";var L=o(9079),N=o(8832),R=o.n(N);o(605);let V=[{id:"2af4a24b-ec0b-42a3-bd7b-270c6cac6ec0",data:{label:"EveryN"},position:{x:0,y:0},is_active:1},{id:"47528905-2dba-4e8b-b50d-3825d0ab1818",data:{label:"Guard"},position:{x:10,y:20},is_active:0},{id:"cc37f26f-6020-4017-bb8c-24fc42bbcd2e",data:{label:"Periodic"},position:{x:20,y:40},is_active:0},{id:"f0b54bee-2d7e-4b8e-b1dc-850d0e577781",data:{label:"Finisher"},position:{x:30,y:60},is_active:0},{id:"6675d729-0911-4f52-9629-c18dd46a613f",data:{label:"Sequence"},position:{x:40,y:80},is_active:0},{id:"67d40469-fe2d-4212-abb1-2c6e48eb9fcb",data:{label:"Idle"},position:{x:50,y:100},is_active:0},{id:"4ef3715d-0d82-4815-ae38-886aedb4f76c",data:{label:"Logging"},position:{x:60,y:120},is_active:1}],F=[{id:"Sequence_connection",source:"6675d729-0911-4f52-9629-c18dd46a613f",target:"47528905-2dba-4e8b-b50d-3825d0ab1818"},{id:"Sequence_connection",source:"6675d729-0911-4f52-9629-c18dd46a613f",target:"cc37f26f-6020-4017-bb8c-24fc42bbcd2e"},{id:"Sequence_connection",source:"6675d729-0911-4f52-9629-c18dd46a613f",target:"f0b54bee-2d7e-4b8e-b1dc-850d0e577781"},{id:"Logging_connection",source:"4ef3715d-0d82-4815-ae38-886aedb4f76c",target:"2af4a24b-ec0b-42a3-bd7b-270c6cac6ec0"},{id:"Logging_connection",source:"4ef3715d-0d82-4815-ae38-886aedb4f76c",target:"6675d729-0911-4f52-9629-c18dd46a613f"},{id:"Logging_connection",source:"4ef3715d-0d82-4815-ae38-886aedb4f76c",target:"67d40469-fe2d-4212-abb1-2c6e48eb9fcb"}],G=(e,t,o)=>{let r=new(R()).graphlib.Graph().setDefaultEdgeLabel(()=>({}));return r.setGraph({rankdir:o.direction}),t.forEach(e=>r.setEdge(e.source,e.target)),e.forEach(e=>{var t,o,s,n;return r.setNode(e.id,{...e,width:null!==(s=null===(t=e.measured)||void 0===t?void 0:t.width)&&void 0!==s?s:0,height:null!==(n=null===(o=e.measured)||void 0===o?void 0:o.height)&&void 0!==n?n:0})}),R().layout(r),{nodes:e.map(e=>{var t,o,s,n;let a=r.node(e.id),i=a.x-(null!==(s=null===(t=e.measured)||void 0===t?void 0:t.width)&&void 0!==s?s:0)/2,l=a.y-(null!==(n=null===(o=e.measured)||void 0===o?void 0:o.height)&&void 0!==n?n:0)/2;return{...e,position:{x:i,y:l}}}),edges:t}},P=()=>{let{fitView:e}=(0,L._K)(),[t,o,s]=(0,L.Rr)(V),[n,a,i]=(0,L.ll)(F),c=(0,l.useCallback)(r=>{console.log(t);let s=G(t,n,{direction:r});o([...s.nodes]),a([...s.edges]),window.requestAnimationFrame(()=>{e()})},[t,n]);return(0,r.jsx)("div",{style:{height:300},children:(0,r.jsx)(L.x$,{nodes:t,edges:n,onNodesChange:s,onEdgesChange:i,defaultEdgeOptions:{animated:!0,style:{stroke:"white"}},style:{backgroundColor:"#D3D2E5"},fitView:!0,connectionLineStyle:{stroke:"white"},children:(0,r.jsxs)(L.s_,{position:"top-right",children:[(0,r.jsx)("button",{onClick:()=>c("TB"),children:" Vertical "}),(0,r.jsx)("button",{onClick:()=>c("LR"),children:" Horizontal "})]})})})};function z(){return(0,r.jsx)(L.tV,{children:(0,r.jsx)(P,{})})}let B=new(o(6731)).EventEmitter;var K=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let o={...e.formData},r=[...e.formElements];return o[t.index]=t.data,r[t.index-3][t.valueName]=t.value,{formElements:r,formData:o};case"userInputNoState":let s={...e.formData};return s[t.index]=t.data,{formElements:[...e.formElements],formData:s};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let a={...e.formData};return t.indices.map(e=>{delete a[e]}),{formData:a,formElements:[...e.formElements]};default:throw Error()}},M=o(3407),A=o(2844);let W={objects:[],objectList:{NONE:-1},noObjects:0,compID:0,targetID:0,treeVisible:!1},H=(0,M.Ue)((0,A.n)(e=>({...W,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),toggleTreeVisibility:()=>e(e=>{e.treeVisible=!e.treeVisible}),setTargetID:t=>e(e=>{e.targetID=t}),addObject:(t,o)=>e(e=>{e.noObjects>0?e.objects=[...e.objects,t]:e.objects=[t],e.objectList[o]=e.noObjects,e.noObjects+=1}),removeObject:t=>e(e=>{if(e.objects&&e.objects.length&&e.noObjects>0&&t>=0){let o=Object.keys(e.objectList).find(o=>e.objectList[o]===Number(t));console.log("Object key is "+o);let{[o]:r,...s}=e.objectList;Object.keys(s).forEach(function(e){s[e]>t&&(console.log("Value "+s[e]+" is greater than removed "+t),s[e]-=1)}),e.objectList=s,e.noObjects-=1,1===e.objects.length?e.objects=[]:e.objects.splice(t,1)}else console.log("Undefined - Cannot Remove"),alert("Cannot Remove Object -> Object is Invalid or Empty")}),setPos:t=>e(e=>{console.log("targetID: "+e.targetID),e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{if(e.objects[e.targetID]){var o=new a.Euler(t.x,t.y,t.z),r=new a.Quaternion().setFromEuler(o);e.objects[e.targetID][e.compID].euler=[t.x,t.y,t.z],e.objects[e.targetID][e.compID].q=[r.x,r.y,r.z,r.w]}}),setColour:t=>e(e=>{e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].color=t)})})));var U=o(8592),Q=function(){return(0,r.jsx)(U.G,{})};function X(e){let{isOpen:t,onClose:o,children:s,setCloseButton:n}=e;return(0,r.jsx)(r.Fragment,{children:t?(0,r.jsxs)("div",{className:"overlay",children:[(0,r.jsx)("div",{className:"overlay_background",onClick:o}),(0,r.jsxs)("div",{className:"overlay_container",children:[n?(0,r.jsx)("div",{className:"overlay_controls",children:(0,r.jsx)("button",{className:"overlay_close_button",type:"button",onClick:o})}):null,s]})]}):null})}o(1472);var Y=o(8154),J=function(){let e=H(e=>e.objects),t=H(e=>e.targetID),o=H(e=>e.compID),s=H(e=>e.noObjects),n=H(e=>e.setObjectVisibility),a=H(e=>e.setTargetID),i=H(e=>e.setPos),c=H(e=>e.setRot),d=H(e=>e.addObject),u=H(e=>e.removeObject),b=H(e=>e.setColour),f=H(e=>e.objectList),h=H(e=>e.toggleTreeVisibility),m=H(e=>e.treeVisible),[j,g]=(0,l.useState)("cylinder"),[p,x]=(0,l.useState)("#f00"),[v,y]=(0,l.useState)("0"),[w,E]=(0,Y.M4)("Object Handler",()=>({ID:{label:"ID:",options:f,onChange:e=>{a(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>n(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>i(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>c(e)},Colour:{value:"#f00",onChange:e=>b(e)},NoObjects:{label:"Total:",value:s},"Remove Object":(0,Y.LI)(()=>{u(t)})}),[t,s]);return(0,l.useEffect)(()=>{E({NoObjects:s}),e[t]&&(E({Visible:e[t][o].display}),E({Position:e[t][o].t}),E({Rotation:e[t][o].euler}),E({Colour:e[t][o].color}))},[s,t,e,E,o]),(0,Y.M4)("Object Creator",()=>({Type:{options:["cylinder","sphere","cube"],onChange:e=>{g(e)}},Colour:{value:"#0f0",onChange:e=>{x(e)}},Key:{value:"obj_001",onChange:e=>{y(e)}},"Add Object":(0,Y.LI)(()=>{v in f?alert("Cannot Create Object -> Key Already in Use"):d([{stype:j,scale:[1,1,1],filename:void 0,radius:.5,length:1,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:p,opacity:0,display:!0,head_length:0,head_radius:0}],v)})}),[s,j,v]),(0,Y.M4)("Behaviour Tree Handler",()=>({[m?"Close Tree Viewer":"Open Tree Viewer"]:(0,Y.LI)(()=>{h()})}),[m]),(0,r.jsx)(r.Fragment,{})};let Z=e=>{let t=(0,l.useRef)(null),o=(0,l.useRef)(),[s,d]=(0,l.useState)(!1),[u,m]=(0,l.useState)(0),[j,g]=(0,l.useState)(!1),[p,x]=(0,l.useState)(!1),[v,y]=(0,l.useReducer)(K,{formData:{},formElements:[]}),w=H(e=>e.addObject),E=H(e=>e.removeObject),S=H(e=>e.objects),_=H(e=>e.treeVisible);return(0,l.useEffect)(()=>{let t=!0;x(!0);let r=e.port,s=window.location.search.substring(1).split("&");if(0===r&&(console.log("WEBSOCKET: port is 0, using server_params..."),r=parseInt(s[0])),null===r||isNaN(r))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+r+"/";o.current=new WebSocket(e),t&&(o.current.onopen=()=>{o.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},o.current.send("Connected"),g(!0)},B.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),o.current.send(e)})),o&&(o.current.onmessage=e=>{let t=JSON.parse(e.data),o=t[0],r=t[1];B.emit("wsRx",o,r)})}},[e.port]),(0,l.useEffect)(()=>{let e={shape:e=>{let t=S.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),e&&e[0].uuid?w(e,e[0].uuid):w(e,t),B.emit("wsSwiftTx",t)},shape_mounted:e=>{let o=e[0],r=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+o);try{var s;let e=0;null===(s=t.current)||void 0===s||s.children[o].children.forEach((t,o)=>{"loaded"===t.name&&e++}),e===r?B.emit("wsSwiftTx","1"):B.emit("wsSwiftTx","0")}catch(e){console.log(e),B.emit("wsSwiftTx","0")}},remove:e=>{console.log("WEBSOCKET: removing id "+e),E(e),B.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(v.formData).length?(B.emit("wsSwiftTx",JSON.stringify(v.formData)),y({type:"reset",indices:Object.keys(v.formData)})):B.emit("wsSwiftTx","[]"),e.forEach(e=>{let o=e[0];e[1].forEach((e,r)=>{var s,n,i;if(null===(s=t.current)||void 0===s?void 0:s.children[o].children[r]){null===(n=t.current)||void 0===n||n.children[o].children[r].position.set(e.t[0],e.t[1],e.t[2]);let s=new a.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(i=t.current)||void 0===i||i.children[o].children[r].setRotationFromQuaternion(s)}})})},sim_time:e=>{m(parseFloat(e))},close:()=>{o.current.close()}};B.removeAllListeners("wsRx"),B.on("wsRx",(t,o)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](o)})},[v,w,E,S]),a.Object3D.DEFAULT_UP=new a.Vector3(0,0,1),(0,r.jsxs)("div",{className:n().visContainer,children:[(0,r.jsx)(X,{isOpen:_,children:(0,r.jsx)(z,{})}),(0,r.jsxs)(i.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,r.jsx)(J,{}),(0,r.jsx)(Q,{}),(0,r.jsx)(b,{t:[1,1,1]}),(0,r.jsx)("hemisphereLight",{groundColor:new a.Color(1118498)}),(0,r.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(c.z,{panSpeed:.5,rotateSpeed:.4}),(0,r.jsx)("axesHelper",{args:[100]}),(0,r.jsx)(f,{}),(0,r.jsx)(l.Suspense,{fallback:null,children:(0,r.jsx)(T,{meshes:S,ref:t})})]})]})};Z.displayName="Viewer";var $=Z},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},1472:function(e){e.exports={overlay_background:"overlay_overlay_background__qcqTT",overlay_container:"overlay_overlay_container__qRGGS"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[471,689,319,776,971,23,744],function(){return e(e.s=8307)}),_N_E=e.O()}]);