(self.webpackChunk_N_E=self.webpackChunk_N_E||[]).push([[931],{8307:function(e,t,s){Promise.resolve().then(s.bind(s,2480)),Promise.resolve().then(s.t.bind(s,2506,23))},2480:function(e,t,s){"use strict";s.d(t,{default:function(){return U}});var o=s(7437),r=s(9449),n=s.n(r),l=s(7776),a=s(7585),i=s(2265),c=s(6547),u=s(6862),d=s(7954);let m=e=>{let{viewport:t,set:s}=(0,u.D)(),{width:r,height:n}=t,l=(0,i.useRef)(null);return(0,o.jsx)(d.c,{makeDefault:!0,ref:l,position:[e.t[0],e.t[1],e.t[2]],near:.01,far:100,fov:70,aspect:n/r})},f=()=>{let{scene:e}=(0,u.D)();return e.background=new l.Color(7895160),e.fog=new l.Fog(.787878,50,60),(0,o.jsxs)("mesh",{receiveShadow:!0,children:[(0,o.jsx)("planeGeometry",{args:[200,200]}),(0,o.jsx)("meshPhongMaterial",{color:4934475,specular:new l.Color(1052688)})]})},h=e=>{let t=(0,i.useRef)(null);return(0,o.jsx)("directionalLight",{ref:t,color:e.color,intensity:e.intensity,position:[e.x,e.y,e.z],castShadow:!0})};var j=s(6080),p=s(2325),g=s(2618),x=s(1957),b=s(7413),y=s(2851);let v=(e,t,s)=>{e.isMesh?(Array.isArray(e.material)?e.material.forEach(e=>{e=e.clone()}):e.material=e.material.clone(),s&&(e.castShadow=!0,e.receiveShadow=!0),1!==t&&(e.material.isMaterial?(e.material.transparent=!0,e.material.opacity=t):Array.isArray(e.material)&&e.material.forEach(e=>{e.transparent=!0,e.opacity=t}))):"PointLight"===e.type?e.visible=!1:(e.isObject3D||e.isGroup)&&e.children.forEach(e=>{v(e,t,s)})},E=e=>{let t=(0,u.H)(b.j,e.url),s=(0,i.useMemo)(()=>t.clone(),[t]);return(0,o.jsxs)("mesh",{position:[e.t[0],e.t[1],e.t[2]],quaternion:[e.q[0],e.q[1],e.q[2],e.q[3]],scale:[e.scale[0],e.scale[1],e.scale[2]],castShadow:!0,receiveShadow:!0,name:"loaded",children:[(0,o.jsx)("primitive",{object:s,attach:"geometry"}),(0,o.jsx)("meshStandardMaterial",{color:e.color?e.color:"hotpink"})]})},w=e=>(0,o.jsx)(i.Fragment,{children:(0,o.jsx)(x.h,{src:e.url,position:[e.t[0],e.t[1],e.t[2]],scale:[e.scale[0],e.scale[1],e.scale[2]],rotation:[e.euler[0],e.euler[1],e.euler[2]],onClick:()=>{console.log(e.id)}})}),O=e=>{let t=(0,u.H)(y.G,e.url),s=(0,i.useMemo)(()=>t.scene.clone(!0),[t.scene]);return(0,i.useEffect)(()=>{s.children.forEach(t=>{v(t,e.opacity,!0)}),s.name="loaded"},[s,e.opacity]),(0,o.jsx)("primitive",{object:s,position:e.t,scale:e.scale,quaternion:e.q})};var S=e=>{if(null===e.filename)return console.log("LOADER: filename is undefined, cannot proceed..."),(0,o.jsx)(o.Fragment,{});{let t=e.filename.split(".").pop(),s=e.filename;switch(s="/retrieve/"+s,null==t?void 0:t.toLowerCase()){case"stl":return(0,o.jsx)(E,{url:s,...e});case"splat":return(0,o.jsx)(w,{url:s,...e});case"ply":return console.log("LOADING PLY FILE HERE"),(0,o.jsx)(w,{url:s,...e});default:return(0,o.jsx)(O,{url:s,...e})}}};let D=e=>{let[t,s]=(0,i.useState)(!1);return(0,i.useEffect)(()=>{s(!0)},[]),(0,o.jsx)(i.Fragment,{children:t&&(0,o.jsx)(i.Suspense,{fallback:(0,o.jsx)(function(){let{progress:e}=(0,j.S)();return(0,o.jsxs)(p.V,{center:!0,children:[Math.round(e)," % loaded"]})},{}),children:(0,o.jsx)(S,{...e})})})},C=e=>{let t=(0,i.useRef)(null);return(0,o.jsx)("mesh",{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],name:"loaded",children:(0,o.jsx)("axesHelper",{args:[e.length?e.length:.1]})})},_=e=>{let t=(0,i.useRef)(null);switch((0,i.useEffect)(()=>{"cylinder"===e.stype&&t.current.rotateX(Math.PI/2)},[e.stype]),e.stype){case"box":default:return(0,o.jsx)("boxGeometry",{args:e.scale?[e.scale[0],e.scale[1],e.scale[2]]:[1,1,1]});case"sphere":return(0,o.jsx)("sphereGeometry",{args:[e.radius,64,64]});case"cylinder":return(0,o.jsx)("cylinderGeometry",{ref:t,args:[e.radius,e.radius,e.length,32]})}},I=e=>{let t=(0,i.useRef)(null);return(0,i.useEffect)(()=>{t.current&&t.current}),(0,o.jsx)(g.Y,{ref:t,position:e.t?[e.t[0],e.t[1],e.t[2]]:[0,0,0],quaternion:e.q?[e.q[0],e.q[1],e.q[2],e.q[3]]:[0,0,0,1],castShadow:!0,name:"loaded",showX:!1,showY:!1,showZ:!1,onClick:()=>{t.current&&t.current},children:(0,o.jsxs)("mesh",{children:[(0,o.jsx)(_,{...e}),(0,o.jsx)("meshStandardMaterial",{transparent:!!e.opacity,color:e.color?e.color:"hotpink",opacity:e.opacity?e.opacity:1})]})})},T=e=>{if(!1==e.display)return(0,o.jsx)(i.Fragment,{});switch(e.stype){case"mesh":case"splat":return(0,o.jsx)(D,{...e});case"axes":return(0,o.jsx)(C,{...e});default:return(0,o.jsx)(I,{...e})}},k=e=>(0,o.jsx)("group",{children:e.meshes.map((e,t)=>(0,o.jsx)(T,{...e},t))}),q=i.forwardRef((e,t)=>(0,o.jsx)("group",{ref:t,children:e.meshes.map((e,t)=>(0,o.jsx)(k,{meshes:e},t))}));q.displayName="GroupCollection";var L=s(9079);function R(){return(0,o.jsx)("div",{style:{height:300},children:(0,o.jsx)(L.x$,{defaultNodes:[{id:"a",type:"input",data:{label:"NO DATA"},position:{x:200,y:10}}],defaultEdges:[{id:"ea-b",source:"a",target:"b"}],defaultEdgeOptions:{animated:!0,style:{stroke:"white"}},fitView:!0,style:{backgroundColor:"#D3D2E5"},connectionLineStyle:{stroke:"white"}})})}s(605);let N=new(s(6731)).EventEmitter;var V=(e,t)=>{switch(console.log("action: ",t,"state ",e),t.type){case"newElement":return{...e,formElements:[...e.formElements,t.data]};case"userInputState":let s={...e.formData},o=[...e.formElements];return s[t.index]=t.data,o[t.index-3][t.valueName]=t.value,{formElements:o,formData:s};case"userInputNoState":let r={...e.formData};return r[t.index]=t.data,{formElements:[...e.formElements],formData:r};case"wsUpdate":let n=[...e.formElements];return n[t.index-3]=t.data,{...e,formElements:n};case"reset":let l={...e.formData};return t.indices.map(e=>{delete l[e]}),{formData:l,formElements:[...e.formElements]};default:throw Error()}},F=s(3407),B=s(2844);let M={objects:[],noObjects:0,compID:0,targetID:0,idList:[],keyList:[],treeVisible:!1},P=(0,F.Ue)((0,B.n)(e=>({...M,setObjectVisibility:t=>e(e=>{e.objects[e.targetID]&&e.objects[e.targetID].map(e=>{e.display=t})}),toggleTreeVisibility:()=>e(e=>{e.treeVisible=!e.treeVisible,console.log("Toggling tree visibility"),console.log(e.treeVisible)}),setTargetID:t=>e(e=>{var s=e.keyList.indexOf(t);console.log("Key: "+t+" has id of "+s),e.targetID=s}),addObject:(t,s)=>e(e=>{e.noObjects>0?(e.objects=[...e.objects,t],e.keyList=[...e.keyList,s],e.idList=[...e.idList,e.noObjects]):(e.objects=[t],e.keyList=[s],e.idList=[e.noObjects]),e.noObjects+=1}),removeObject:t=>e(e=>{var s=e.keyList.indexOf(t);console.log("ID from list is "+t+" with index "+s),console.log("Number of Objects BEFORE removal: "+e.noObjects),console.log("objects state BEFORE removal: "+e.objects),console.log("idList state BEFORE removal: "+e.keyList),e.objects&&e.objects.length&&e.noObjects>0?(console.log("Object is Valid and In List"),console.log("Deleting object "+e.objects[s].stype)):console.log("Undefined - Cannot Remove")}),setPos:t=>e(e=>{e.objects[e.targetID]&&(e.objects[e.targetID][e.compID].t=[t.x,t.y,t.z])}),setRot:t=>e(e=>{if(e.objects[e.targetID]){var s=new l.Euler(t.x,t.y,t.z),o=new l.Quaternion().setFromEuler(s);e.objects[e.targetID][e.compID].euler=[t.x,t.y,t.z],e.objects[e.targetID][e.compID].q=[o.x,o.y,o.z,o.w]}})})));var z=s(8592),A=function(){return(0,o.jsx)(z.G,{})};function G(e){let{isOpen:t,onClose:s,children:r,setCloseButton:n}=e;return(0,o.jsx)(o.Fragment,{children:t?(0,o.jsxs)("div",{className:"overlay",children:[(0,o.jsx)("div",{className:"overlay_background",onClick:s}),(0,o.jsxs)("div",{className:"overlay_container",children:[n?(0,o.jsx)("div",{className:"overlay_controls",children:(0,o.jsx)("button",{className:"overlay_close_button",type:"button",onClick:s})}):null,r]})]}):null})}s(1472);var K=s(8154),W=function(){let e=P(e=>e.objects),t=P(e=>e.targetID),s=P(e=>e.compID),r=P(e=>e.noObjects),n=P(e=>e.setObjectVisibility),l=P(e=>e.setTargetID),a=P(e=>e.setPos),c=P(e=>e.setRot),u=P(e=>e.addObject),d=P(e=>e.removeObject),m=P(e=>e.idList),f=P(e=>e.toggleTreeVisibility),h=P(e=>e.treeVisible),[j,p]=(0,i.useState)("cylinder"),[g,x]=(0,i.useState)("#f00"),[b,y]=(0,i.useState)("0"),[v,E]=(0,K.M4)("Object Handler",()=>({ID:{label:"ID:",options:m,onChange:e=>{l(e)}},Visible:{label:"Visible:",value:!0,onChange:e=>n(e)},Position:{label:"Position:",value:{x:0,y:0,z:0},onChange:e=>a(e)},Rotation:{label:"Rotation:",value:{x:0,y:0,z:0},onChange:e=>c(e)},NoObjects:{label:"Total:",value:r},"Remove Object":(0,K.LI)(()=>{d(t)})}),[m,t]);return(0,i.useEffect)(()=>{E({NoObjects:r}),e[t]&&(E({Visible:e[t][s].display}),E({Position:e[t][s].t}),E({Rotation:e[t][s].euler}),E({ID:m}))},[r,t,e,E,s,m]),(0,K.M4)("Object Creator",()=>({Type:{options:["cylinder","sphere","cube"],onChange:e=>{p(e)}},Colour:{value:"#f00",onChange:e=>{x(e)}},ID:{value:"",onChange:e=>{y(e)}},"Add Object":(0,K.LI)(()=>{u([{stype:j,scale:[1,1,1],filename:void 0,radius:.5,length:1,euler:[0,0,0],q:[0,0,0,1],t:[0,0,0],v:[0,0,0],color:g,opacity:0,display:!0,head_length:0,head_radius:0}],b)})}),[r,j,g,b]),(0,K.M4)("Behaviour Tree Handler",()=>({[h?"Close Tree Viewer":"Open Tree Viewer"]:(0,K.LI)(()=>{f()})}),[h]),(0,o.jsx)(o.Fragment,{})};let H=e=>{let t=(0,i.useRef)(null),s=(0,i.useRef)(),[r,u]=(0,i.useState)(!1),[d,j]=(0,i.useState)(0),[p,g]=(0,i.useState)(!1),[x,b]=(0,i.useState)(!1),[y,v]=(0,i.useReducer)(V,{formData:{},formElements:[]}),E=P(e=>e.addObject),w=P(e=>e.removeObject),O=P(e=>e.objects),S=P(e=>e.treeVisible);return(0,i.useEffect)(()=>{let t=!0;b(!0);let o=e.port,r=window.location.search.substring(1).split("&");if(0===o&&(console.log("WEBSOCKET: port is 0, using server_params..."),o=parseInt(r[0])),null===o||isNaN(o))console.log("WEBSOCKET: port is null or NaN, cannot establish socket"),t=!1;else{let e="ws://localhost:"+o+"/";s.current=new WebSocket(e),t&&(s.current.onopen=()=>{s.current.onclose=()=>{console.log("WEBSOCKET: closed by Swift"),setTimeout(()=>{window.close()},5e3)},s.current.send("Connected"),g(!0)},N.on("wsSwiftTx",e=>{console.log("WEBSOCKET: sending back: "+e),s.current.send(e)})),s&&(s.current.onmessage=e=>{let t=JSON.parse(e.data),s=t[0],o=t[1];N.emit("wsRx",s,o)})}},[e.port]),(0,i.useEffect)(()=>{let e={shape:e=>{let t=O.length.toString();console.log("WEBSOCKET: requested shape function. ID: "+t),console.log("WEBSOCKET: data: "),console.log(e),E(e,t),N.emit("wsSwiftTx",t)},shape_mounted:e=>{let s=e[0],o=e[1];console.log("WEBSOCKET: requested shape mounted function. ID: "+s);try{var r;let e=0;null===(r=t.current)||void 0===r||r.children[s].children.forEach((t,s)=>{"loaded"===t.name&&e++}),e===o?N.emit("wsSwiftTx","1"):N.emit("wsSwiftTx","0")}catch(e){console.log(e),N.emit("wsSwiftTx","0")}},remove:e=>{console.log("WEBSOCKET: removing id "+e),w(e),N.emit("wsSwiftTx","0")},shape_poses:e=>{0!==Object.keys(y.formData).length?(N.emit("wsSwiftTx",JSON.stringify(y.formData)),v({type:"reset",indices:Object.keys(y.formData)})):N.emit("wsSwiftTx","[]"),e.forEach(e=>{let s=e[0];e[1].forEach((e,o)=>{var r,n,a;if(null===(r=t.current)||void 0===r?void 0:r.children[s].children[o]){null===(n=t.current)||void 0===n||n.children[s].children[o].position.set(e.t[0],e.t[1],e.t[2]);let r=new l.Quaternion(e.q[0],e.q[1],e.q[2],e.q[3]);null===(a=t.current)||void 0===a||a.children[s].children[o].setRotationFromQuaternion(r)}})})},sim_time:e=>{j(parseFloat(e))},close:()=>{s.current.close()}};N.removeAllListeners("wsRx"),N.on("wsRx",(t,s)=>{console.log("WEBSOCKET: running desired function..."),console.log(t),e[t](s)})},[y,E,w,O]),l.Object3D.DEFAULT_UP=new l.Vector3(0,0,1),(0,o.jsxs)("div",{className:n().visContainer,children:[(0,o.jsx)(G,{isOpen:S,children:(0,o.jsx)(R,{})}),(0,o.jsxs)(a.Xz,{gl:{antialias:!0,preserveDrawingBuffer:!0},id:"threeCanvas",children:[(0,o.jsx)(W,{}),(0,o.jsx)(A,{}),(0,o.jsx)(m,{t:[1,1,1]}),(0,o.jsx)("hemisphereLight",{groundColor:new l.Color(1118498)}),(0,o.jsx)(h,{x:10,y:10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(h,{x:-10,y:-10,z:10,color:16777215,intensity:.2}),(0,o.jsx)(c.z,{panSpeed:.5,rotateSpeed:.4}),(0,o.jsx)("axesHelper",{args:[100]}),(0,o.jsx)(f,{}),(0,o.jsx)(i.Suspense,{fallback:null,children:(0,o.jsx)(q,{meshes:O,ref:t})})]})]})};H.displayName="Viewer";var U=H},2506:function(e){e.exports={container:"home_container__TLSt1",main:"home_main__C5E0Z"}},1472:function(e){e.exports={overlay_background:"overlay_overlay_background__qcqTT",overlay_container:"overlay_overlay_container__qRGGS"}},9449:function(e){e.exports={visContainer:"vis_visContainer__9c9Sa"}}},function(e){e.O(0,[471,689,319,875,971,23,744],function(){return e(e.s=8307)}),_N_E=e.O()}]);