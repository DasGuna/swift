(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{8307:function(e,t,o){Promise.resolve().then(o.bind(o,2480)),Promise.resolve().then(o.t.bind(o,2506,23))},2480:function(e,t,o){"use strict";o.d(t,{default:function(){return U}});var s=o(7437),r=o(9449),n=o.n(r),l=o(7776),a=o(7585),i=o(2265),c=o(6547),u=o(6862),d=o(7954);let m=e=>{let{viewport:t,set:o}=(0,u.D)(),{width:r,height:n}=t,l=(0,i.useRef)(null);return(0,s.jsx)(d.c,{makeDefault:!0,ref:l,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/r})},f=()=>{let{scene:e}=(0,u.D)();return e.background=new l.Color(7895160),e.fog=new l.Fog(.787878,50,60),(0,s.jsxs)("mesh",{receiveShadow:!0,children:[(0,s.jsx)("planeGeometry",{args:[200,200]}),(0,s.jsx)("meshPhongMaterial",{color:4934475,specular:new l.Color(1052688)})]})},j=e=>{let t=(0,i.useRef)(null);return(0,s.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var p=o(6080),h=o(2325),g=o(2618),b=o(1957),x=o(7413),y=o(2851);let v=(e,t,o)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),o&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{v(e,t,o)})},E=e=>{let t=(0,u.H)(x.j,e.url),o=(0,i.useMemo)(()=>t.clone(),[t]);return(0,s.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,s.jsx)("primitive",{object:o,attach:"geometry"}),(0,s.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},w=e=>(0,s.jsx)(i.Fragment,{children:(0,s.jsx)(b.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),O=e=>{let t=(0,u.H)(y.G,e.url),o=(0,i.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,i.useEffect)(()=>{o.children.forEach(t=>{v(t,e.opacity,!0)}),o.name="loaded"},[o,e.opacity]),(0,s.jsx)("primitive",{object:o,position:e.t,scale:e.scale,quaternion:e.q})};var S=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,s.jsx)(s.Fragment,{});{let t=e.filename.split(".").pop(),o=e.filename;switch(o="/retrieve/"+o,null==t?void 0:t.toLowerCase()){case"stl":return(0,s.jsx)(E,{url:o,...e});case"splat":return(0,s.jsx)(w,{url:o,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,s.jsx)(w,{url:o,...e});default:return(0,s.jsx)(O,{url:o,...e})}}};let D=e=>{let[t,o]=(0,i.useState)(!1);return(0,i.useEffect)(()=>{o(!0)},[]),(0,s.jsx)(i.Fragment,{children:t&&(0,s.jsx)(i.Suspense,{fallback:(0,s.jsx)(function(){let{progress:e}=(0,p.S)();return(0,s.jsxs)(h.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,s.jsx)(S,{...e})})})},C=e=>{let t=(0,i.useRef)(null);return(0,s.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,s.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},I=e=>{let t=(0,i.useRef)(null);switch((0,i.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,s.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,s.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,s.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},_=e=>{let t=(0,i.useRef)(null);return(0,i.useEffect)(()=>{t.current&&t.current}),(0,s.jsx)(g.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,s.jsxs)("mesh",{children:[(0,s.jsx)(I,{...e}),(0,s.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},T=e=>{if(!1==e.display)return(0,s.jsx)(i.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,s.jsx)(D,{...e});case"axes":return(0,s.jsx)(C,{...e});default:return(0,s.jsx)(_,{...e})}},k=e=>(0,s.jsx)("group",{children:e.meshes.map((e,t)=>(0,s.jsx)(T,{...e},t))}),q=i.forwardRef((e,t)=>(0,s.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,s.jsx)(k,{meshes:e},t))}));q.displayName="GroupCollection";var N=o(9079);function R(){return(0,s.jsx)("div",{style:{height:300},children:(0,s.jsx)(N.x$,{defaultNodes:[{id:"a",type:"input",data:{label:"NO DATA"},position:{x:200,y:10}}],defaultEdges:[{id:"ea-b",source:"a",target:"b"}],defaultEdgeOptions:{animated:!0,style:{stroke:"white"}},fitView:!0,style:{backgroundColor:"#D3D2E5"},connectionLineStyle:{stroke:"white"}})})}o(605);let L=new(o(6731)).EventEmitter;var V=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let o={...e.formData},s=[...e.formElements];return o[t.index]=t.data,s[t.index-3][t.valueName]=t.value,{formElements:s,formData:o};case"userInputNoState":let r={...e.formData};return r[t.index]=t.data,{formElements:[...e.formElements],formData:r};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let l={...e.formData};return t.indices.map(e=>{delete l[e]}),{formData:l,formElements:[...e.formElements]};default:throw Error()}},F=o(3407),B=o(2844);let A={objects:[],noObjects:0,compID:0,targetID:0,idList:[],keyList:["NONE"],temp:{NONE:-1},treeVisible:!1},G=(0,F.Ue)((0,B.n)(e=>({...A,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),toggleTreeVisibility:()=>e(e=>{e.treeVisible=!e.treeVisible}),setTargetID:t=>e(e=>{e.targetID=t}),addObject:(t,o)=>e(e=>{e.noObjects>0?e.objects=[...e.objects,t]:e.objects=[t],e.temp[o]=e.noObjects,e.noObjects+=1}),removeObject:t=>e(e=>{for(let[o,s]of(console.log("Given idx is "+t),console.log("Number of Objects BEFORE removal: "+e.noObjects),console.log("objects state BEFORE removal: "+e.objects),console.log("Temp BEFORE removal"),Object.entries(e.temp)))console.log(s);if(e.objects&&e.objects.length&&e.noObjects>0){console.log("Object is Valid and In List");let o=Object.keys(e.temp).find(o=>e.temp[o]===t);console.log("Object key is "+o);let{[o]:s,...r}=e.temp;for(let[e,t]of(console.log("Temp post removal"),Object.entries(r)))console.log(t)}else console.log("Undefined - Cannot Remove")}),setPos:t=>e(e=>{console.log("targetID: "+e.targetID),e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{if(e.objects[e.targetID]){var o=new l.Euler(t.x,t.y,t.z),s=new l.Quaternion().setFromEuler(o);e.objects[e.targetID][e.compID].euler=[t.x,t.y,t.z],e.objects[e.targetID][e.compID].q=[s.x,s.y,s.z,s.w]}}),setColour:t=>e(e=>{e.objects[e.targetID]&&(console.log("Current colour of object: "+e.objects[e.targetID][e.compID].color),console.log("Input colour: "+t),e.objects[e.targetID][e.compID].color=t)})})));var K=o(8592),M=function(){return(0,s.jsx)(K.G,{})};function P(e){let{isOpen:t,onClose:o,children:r,setCloseButton:n}=e;return(0,s.jsx)(s.Fragment,{children:t?(0,s.jsxs)("div",{className:"overlay",children:[(0,s.jsx)("div",{className:"overlay_background",onClick:o}),(0,s.jsxs)("div",{className:"overlay_container",children:[n?(0,s.jsx)("div",{className:"overlay_controls",children:(0,s.jsx)("button",{className:"overlay_close_button",type:"button",onClick:o})}):null,r]})]}):null})}o(1472);var z=o(8154),W=function(){let e=G(e=>e.objects),t=G(e=>e.targetID),o=G(e=>e.compID),r=G(e=>e.noObjects),n=G(e=>e.setObjectVisibility),l=G(e=>e.setTargetID),a=G(e=>e.setPos),c=G(e=>e.setRot),u=G(e=>e.addObject),d=G(e=>e.removeObject),m=G(e=>e.setColour);G(e=>e.keyList);let f=G(e=>e.temp),j=G(e=>e.toggleTreeVisibility),p=G(e=>e.treeVisible),[h,g]=(0,i.useState)("cylinder"),[b,x]=(0,i.useState)("#f00"),[y,v]=(0,i.useState)("0"),[E,w]=(0,z.M4)("Object Handler",()=>({ID:{label:"ID:",options:f,onChange:e=>{l(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>n(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>a(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>c(e)},Colour:{value:"#f00",onChange:e=>m(e)},NoObjects:{label:"Total:",value:r},"Remove Object":(0,z.LI)(()=>{d(t)})}),[t,r]);return(0,i.useEffect)(()=>{w({NoObjects:r}),e[t]&&(w({Visible:e[t][o].display}),w({Position:e[t][o].t}),w({Rotation:e[t][o].euler}),w({Colour:e[t][o].color}))},[r,t,e,w,o]),(0,z.M4)("Object Creator",()=>({Type:{options:["cylinder","sphere","cube"],onChange:e=>{g(e),console.log("Type Set "+e)}},Colour:{value:"#0f0",onChange:e=>{x(e),console.log("Colour Set "+e)}},Key:{value:"obj_001",onChange:e=>{v(e),console.log("Key Set "+e)}},"Add Object":(0,z.LI)(()=>{console.log("object type: "+h),console.log("object colour: "+b),console.log("object key: "+y),y in f?alert("Cannot Create Object; Key Already in Use"):u([{stype:h,scale:[1,1,1],filename:void 0,radius:.5,length:1,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:b,opacity:0,display:!0,head_length:0,head_radius:0}],y)})}),[r,h,y]),(0,z.M4)("Behaviour Tree Handler",()=>({[p?"Close Tree Viewer":"Open Tree Viewer"]:(0,z.LI)(()=>{j()})}),[p]),(0,s.jsx)(s.Fragment,{})};let H=e=>{let t=(0,i.useRef)(null),o=(0,i.useRef)(),[r,u]=(0,i.useState)(!1),[d,p]=(0,i.useState)(0),[h,g]=(0,i.useState)(!1),[b,x]=(0,i.useState)(!1),[y,v]=(0,i.useReducer)(V,{formData:{},formElements:[]}),E=G(e=>e.addObject),w=G(e=>e.removeObject),O=G(e=>e.objects),S=G(e=>e.treeVisible);return(0,i.useEffect)(()=>{let t=!0;x(!0);let s=e.port,r=window.location.search.substring(1).split("&");if(0===s&&(console.log("WEBSOCKET: port is 0, using server_params..."),s=parseInt(r[0])),null===s||isNaN(s))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+s+"/";o.current=new WebSocket(e),t&&(o.current.onopen=()=>{o.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},o.current.send("Connected"),g(!0)},L.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),o.current.send(e)})),o&&(o.current.onmessage=e=>{let t=JSON.parse(e.data),o=t[0],s=t[1];L.emit("wsRx",o,s)})}},[e.port]),(0,i.useEffect)(()=>{let e={shape:e=>{let t=O.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),E(e,t),L.emit("wsSwiftTx",t)},shape_mounted:e=>{let o=e[0],s=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+o);try{var r;let e=0;null===(r=t.current)||void 0===r||r.children[o].children.forEach((t,o)=>{"loaded"===t.name&&e++}),e===s?L.emit("wsSwiftTx","1"):L.emit("wsSwiftTx","0")}catch(e){console.log(e),L.emit("wsSwiftTx","0")}},remove:e=>{console.log("WEBSOCKET: removing id "+e),w(e),L.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(y.formData).length?(L.emit("wsSwiftTx",JSON.stringify(y.formData)),v({type:"reset",indices:Object.keys(y.formData)})):L.emit("wsSwiftTx","[]"),e.forEach(e=>{let o=e[0];e[1].forEach((e,s)=>{var r,n,a;if(null===(r=t.current)||void 0===r?void 0:r.children[o].children[s]){null===(n=t.current)||void 0===n||n.children[o].children[s].position.set(e.t[0],e.t[1],e.t[2]);let r=new l.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(a=t.current)||void 0===a||a.children[o].children[s].setRotationFromQuaternion(r)}})})},sim_time:e=>{p(parseFloat(e))},close:()=>{o.current.close()}};L.removeAllListeners("wsRx"),L.on("wsRx",(t,o)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](o)})},[y,E,w,O]),l.Object3D.DEFAULT_UP=new l.Vector3(0,0,1),(0,s.jsxs)("div",{className:n().visContainer,children:[(0,s.jsx)(P,{isOpen:S,children:(0,s.jsx)(R,{})}),(0,s.jsxs)(a.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,s.jsx)(W,{}),(0,s.jsx)(M,{}),(0,s.jsx)(m,{t:[1,1,1]}),(0,s.jsx)("hemisphereLight",{groundColor:new l.Color(1118498)}),(0,s.jsx)(j,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,s.jsx)(j,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,s.jsx)(c.z,{panSpeed:.5,rotateSpeed:.4}),(0,s.jsx)("axesHelper",{args:[100]}),(0,s.jsx)(f,{}),(0,s.jsx)(i.Suspense,{fallback:null,children:(0,s.jsx)(q,{meshes:O,ref:t})})]})]})};H.displayName="Viewer";var U=H},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},1472:function(e){e.exports={overlay_background:"overlay_overlay_background__qcqTT",overlay_container:"overlay_overlay_container__qRGGS"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[471,689,319,875,971,23,744],function(){return e(e.s=8307)}),_N_E=e.O()}]);