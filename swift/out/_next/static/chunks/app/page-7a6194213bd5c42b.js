(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{8307:function(e,t,s){Promise.resolve().then(s.bind(s,2480)),Promise.resolve().then(s.t.bind(s,2506,23))},2480:function(e,t,s){"use strict";s.d(t,{default:function(){return U}});var r=s(7437),o=s(9449),n=s.n(o),a=s(7776),l=s(7585),i=s(2265),c=s(6547),u=s(6862),d=s(7954);let m=e=>{let{viewport:t,set:s}=(0,u.D)(),{width:o,height:n}=t,a=(0,i.useRef)(null);return(0,r.jsx)(d.c,{makeDefault:!0,ref:a,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/o})},f=()=>{let{scene:e}=(0,u.D)();return e.background=new a.Color(7895160),e.fog=new a.Fog(.787878,50,60),(0,r.jsxs)("mesh",{receiveShadow:!0,children:[(0,r.jsx)("planeGeometry",{args:[200,200]}),(0,r.jsx)("meshPhongMaterial",{color:4934475,specular:new a.Color(1052688)})]})},j=e=>{let t=(0,i.useRef)(null);return(0,r.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var h=s(6080),p=s(2325),b=s(2618),g=s(1957),x=s(7413),y=s(2851);let v=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{v(e,t,s)})},E=e=>{let t=(0,u.H)(x.j,e.url),s=(0,i.useMemo)(()=>t.clone(),[t]);return(0,r.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,r.jsx)("primitive",{object:s,attach:"geometry"}),(0,r.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},w=e=>(0,r.jsx)(i.Fragment,{children:(0,r.jsx)(g.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),O=e=>{let t=(0,u.H)(y.G,e.url),s=(0,i.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,i.useEffect)(()=>{s.children.forEach(t=>{v(t,e.opacity,!0)}),s.name="loaded"},[s,e.opacity]),(0,r.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var S=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,r.jsx)(r.Fragment,{});{let t=e.filename.split(".").pop(),s=e.filename;switch(s="/retrieve/"+s,null==t?void 0:t.toLowerCase()){case"stl":return(0,r.jsx)(E,{url:s,...e});case"splat":return(0,r.jsx)(w,{url:s,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,r.jsx)(w,{url:s,...e});default:return(0,r.jsx)(O,{url:s,...e})}}};let C=e=>{let[t,s]=(0,i.useState)(!1);return(0,i.useEffect)(()=>{s(!0)},[]),(0,r.jsx)(i.Fragment,{children:t&&(0,r.jsx)(i.Suspense,{fallback:(0,r.jsx)(function(){let{progress:e}=(0,h.S)();return(0,r.jsxs)(p.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,r.jsx)(S,{...e})})})},D=e=>{let t=(0,i.useRef)(null);return(0,r.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,r.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},_=e=>{let t=(0,i.useRef)(null);switch((0,i.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,r.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,r.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,r.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},I=e=>{let t=(0,i.useRef)(null);return(0,i.useEffect)(()=>{t.current&&t.current}),(0,r.jsx)(b.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,r.jsxs)("mesh",{children:[(0,r.jsx)(_,{...e}),(0,r.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},T=e=>{if(!1==e.display)return(0,r.jsx)(i.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,r.jsx)(C,{...e});case"axes":return(0,r.jsx)(D,{...e});default:return(0,r.jsx)(I,{...e})}},q=e=>(0,r.jsx)("group",{children:e.meshes.map((e,t)=>(0,r.jsx)(T,{...e},t))}),k=i.forwardRef((e,t)=>(0,r.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,r.jsx)(q,{meshes:e},t))}));k.displayName="GroupCollection";var N=s(9079);function R(){return(0,r.jsx)("div",{style:{height:300},children:(0,r.jsx)(N.x$,{defaultNodes:[{id:"71f8adf6-ef0f-4a92-b8d4-48f5c692c737",type:"input",data:{label:"EveryN"},position:{x:200,y:10}}],defaultEdges:[{id:"ea-b",source:"a",target:"b"}],defaultEdgeOptions:{animated:!0,style:{stroke:"white"}},fitView:!0,style:{backgroundColor:"#D3D2E5"},connectionLineStyle:{stroke:"white"}})})}s(605);let L=new(s(6731)).EventEmitter;var V=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},r=[...e.formElements];return s[t.index]=t.data,r[t.index-3][t.valueName]=t.value,{formElements:r,formData:s};case"userInputNoState":let o={...e.formData};return o[t.index]=t.data,{formElements:[...e.formElements],formData:o};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let a={...e.formData};return t.indices.map(e=>{delete a[e]}),{formData:a,formElements:[...e.formElements]};default:throw Error()}},F=s(3407),M=s(2844);let P={objects:[],objectList:{NONE:-1},noObjects:0,compID:0,targetID:0,treeVisible:!1},z=(0,F.Ue)((0,M.n)(e=>({...P,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),toggleTreeVisibility:()=>e(e=>{e.treeVisible=!e.treeVisible}),setTargetID:t=>e(e=>{e.targetID=t}),addObject:(t,s)=>e(e=>{e.noObjects>0?e.objects=[...e.objects,t]:e.objects=[t],e.objectList[s]=e.noObjects,e.noObjects+=1}),removeObject:t=>e(e=>{if(e.objects&&e.objects.length&&e.noObjects>0&&t>=0){let s=Object.keys(e.objectList).find(s=>e.objectList[s]===Number(t));console.log("Object key is "+s);let{[s]:r,...o}=e.objectList;Object.keys(o).forEach(function(e){o[e]>t&&(console.log("Value "+o[e]+" is greater than removed "+t),o[e]-=1)}),e.objectList=o,e.noObjects-=1,1===e.objects.length?e.objects=[]:e.objects.splice(t,1)}else console.log("Undefined - Cannot Remove"),alert("Cannot Remove Object -> Object is Invalid or Empty")}),setPos:t=>e(e=>{console.log("targetID: "+e.targetID),e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{if(e.objects[e.targetID]){var s=new a.Euler(t.x,t.y,t.z),r=new a.Quaternion().setFromEuler(s);e.objects[e.targetID][e.compID].euler=[t.x,t.y,t.z],e.objects[e.targetID][e.compID].q=[r.x,r.y,r.z,r.w]}}),setColour:t=>e(e=>{e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].color=t)})})));var B=s(8592),G=function(){return(0,r.jsx)(B.G,{})};function K(e){let{isOpen:t,onClose:s,children:o,setCloseButton:n}=e;return(0,r.jsx)(r.Fragment,{children:t?(0,r.jsxs)("div",{className:"overlay",children:[(0,r.jsx)("div",{className:"overlay_background",onClick:s}),(0,r.jsxs)("div",{className:"overlay_container",children:[n?(0,r.jsx)("div",{className:"overlay_controls",children:(0,r.jsx)("button",{className:"overlay_close_button",type:"button",onClick:s})}):null,o]})]}):null})}s(1472);var A=s(8154),W=function(){let e=z(e=>e.objects),t=z(e=>e.targetID),s=z(e=>e.compID),o=z(e=>e.noObjects),n=z(e=>e.setObjectVisibility),a=z(e=>e.setTargetID),l=z(e=>e.setPos),c=z(e=>e.setRot),u=z(e=>e.addObject),d=z(e=>e.removeObject),m=z(e=>e.setColour),f=z(e=>e.objectList),j=z(e=>e.toggleTreeVisibility),h=z(e=>e.treeVisible),[p,b]=(0,i.useState)("cylinder"),[g,x]=(0,i.useState)("#f00"),[y,v]=(0,i.useState)("0"),[E,w]=(0,A.M4)("Object Handler",()=>({ID:{label:"ID:",options:f,onChange:e=>{a(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>n(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>l(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>c(e)},Colour:{value:"#f00",onChange:e=>m(e)},NoObjects:{label:"Total:",value:o},"Remove Object":(0,A.LI)(()=>{d(t)})}),[t,o]);return(0,i.useEffect)(()=>{w({NoObjects:o}),e[t]&&(w({Visible:e[t][s].display}),w({Position:e[t][s].t}),w({Rotation:e[t][s].euler}),w({Colour:e[t][s].color}))},[o,t,e,w,s]),(0,A.M4)("Object Creator",()=>({Type:{options:["cylinder","sphere","cube"],onChange:e=>{b(e)}},Colour:{value:"#0f0",onChange:e=>{x(e)}},Key:{value:"obj_001",onChange:e=>{v(e)}},"Add Object":(0,A.LI)(()=>{y in f?alert("Cannot Create Object -> Key Already in Use"):u([{stype:p,scale:[1,1,1],filename:void 0,radius:.5,length:1,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:g,opacity:0,display:!0,head_length:0,head_radius:0}],y)})}),[o,p,y]),(0,A.M4)("Behaviour Tree Handler",()=>({[h?"Close Tree Viewer":"Open Tree Viewer"]:(0,A.LI)(()=>{j()})}),[h]),(0,r.jsx)(r.Fragment,{})};let H=e=>{let t=(0,i.useRef)(null),s=(0,i.useRef)(),[o,u]=(0,i.useState)(!1),[d,h]=(0,i.useState)(0),[p,b]=(0,i.useState)(!1),[g,x]=(0,i.useState)(!1),[y,v]=(0,i.useReducer)(V,{formData:{},formElements:[]}),E=z(e=>e.addObject),w=z(e=>e.removeObject),O=z(e=>e.objects),S=z(e=>e.treeVisible);return(0,i.useEffect)(()=>{let t=!0;x(!0);let r=e.port,o=window.location.search.substring(1).split("&");if(0===r&&(console.log("WEBSOCKET: port is 0, using server_params..."),r=parseInt(o[0])),null===r||isNaN(r))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+r+"/";s.current=new WebSocket(e),t&&(s.current.onopen=()=>{s.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},s.current.send("Connected"),b(!0)},L.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),s.current.send(e)})),s&&(s.current.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],r=t[1];L.emit("wsRx",s,r)})}},[e.port]),(0,i.useEffect)(()=>{let e={shape:e=>{let t=O.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),e&&e[0].uuid?E(e,e[0].uuid):E(e,t),L.emit("wsSwiftTx",t)},shape_mounted:e=>{let s=e[0],r=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+s);try{var o;let e=0;null===(o=t.current)||void 0===o||o.children[s].children.forEach((t,s)=>{"loaded"===t.name&&e++}),e===r?L.emit("wsSwiftTx","1"):L.emit("wsSwiftTx","0")}catch(e){console.log(e),L.emit("wsSwiftTx","0")}},remove:e=>{console.log("WEBSOCKET: removing id "+e),w(e),L.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(y.formData).length?(L.emit("wsSwiftTx",JSON.stringify(y.formData)),v({type:"reset",indices:Object.keys(y.formData)})):L.emit("wsSwiftTx","[]"),e.forEach(e=>{let s=e[0];e[1].forEach((e,r)=>{var o,n,l;if(null===(o=t.current)||void 0===o?void 0:o.children[s].children[r]){null===(n=t.current)||void 0===n||n.children[s].children[r].position.set(e.t[0],e.t[1],e.t[2]);let o=new a.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(l=t.current)||void 0===l||l.children[s].children[r].setRotationFromQuaternion(o)}})})},sim_time:e=>{h(parseFloat(e))},close:()=>{s.current.close()}};L.removeAllListeners("wsRx"),L.on("wsRx",(t,s)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](s)})},[y,E,w,O]),a.Object3D.DEFAULT_UP=new a.Vector3(0,0,1),(0,r.jsxs)("div",{className:n().visContainer,children:[(0,r.jsx)(K,{isOpen:S,children:(0,r.jsx)(R,{})}),(0,r.jsxs)(l.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,r.jsx)(W,{}),(0,r.jsx)(G,{}),(0,r.jsx)(m,{t:[1,1,1]}),(0,r.jsx)("hemisphereLight",{groundColor:new a.Color(1118498)}),(0,r.jsx)(j,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(j,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,r.jsx)(c.z,{panSpeed:.5,rotateSpeed:.4}),(0,r.jsx)("axesHelper",{args:[100]}),(0,r.jsx)(f,{}),(0,r.jsx)(i.Suspense,{fallback:null,children:(0,r.jsx)(k,{meshes:O,ref:t})})]})]})};H.displayName="Viewer";var U=H},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},1472:function(e){e.exports={overlay_background:"overlay_overlay_background__qcqTT",overlay_container:"overlay_overlay_container__qRGGS"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[471,689,319,875,971,23,744],function(){return e(e.s=8307)}),_N_E=e.O()}]);