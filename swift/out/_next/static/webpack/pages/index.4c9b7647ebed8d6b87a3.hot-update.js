webpackHotUpdate_N_E("pages/index",{

/***/ "./components/SwiftComponents.tsx":
/*!****************************************!*\
  !*** ./components/SwiftComponents.tsx ***!
  \****************************************/
/*! exports provided: Plane, ShadowedLight, Camera, Shape */
/***/ (function(module, __webpack_exports__, __webpack_require__) {

"use strict";
eval("__webpack_require__.r(__webpack_exports__);\n/* WEBPACK VAR INJECTION */(function(module) {/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"Plane\", function() { return Plane; });\n/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"ShadowedLight\", function() { return ShadowedLight; });\n/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"Camera\", function() { return Camera; });\n/* harmony export (binding) */ __webpack_require__.d(__webpack_exports__, \"Shape\", function() { return Shape; });\n/* harmony import */ var react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__ = __webpack_require__(/*! react/jsx-dev-runtime */ \"./node_modules/react/jsx-dev-runtime.js\");\n/* harmony import */ var react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0___default = /*#__PURE__*/__webpack_require__.n(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__);\n/* harmony import */ var _home_jesse_python_swift_next_swift_node_modules_babel_runtime_helpers_esm_defineProperty__WEBPACK_IMPORTED_MODULE_1__ = __webpack_require__(/*! ./node_modules/@babel/runtime/helpers/esm/defineProperty */ \"./node_modules/@babel/runtime/helpers/esm/defineProperty.js\");\n/* harmony import */ var react__WEBPACK_IMPORTED_MODULE_2__ = __webpack_require__(/*! react */ \"./node_modules/react/index.js\");\n/* harmony import */ var react__WEBPACK_IMPORTED_MODULE_2___default = /*#__PURE__*/__webpack_require__.n(react__WEBPACK_IMPORTED_MODULE_2__);\n/* harmony import */ var react_three_fiber__WEBPACK_IMPORTED_MODULE_3__ = __webpack_require__(/*! react-three-fiber */ \"./node_modules/react-three-fiber/web.js\");\n/* harmony import */ var three__WEBPACK_IMPORTED_MODULE_4__ = __webpack_require__(/*! three */ \"./node_modules/three/build/three.module.js\");\n\n\n\n\nvar _jsxFileName = \"/home/jesse/python/swift/next-swift/components/SwiftComponents.tsx\",\n    _this = undefined,\n    _s = $RefreshSig$(),\n    _s2 = $RefreshSig$(),\n    _s3 = $RefreshSig$(),\n    _s4 = $RefreshSig$(),\n    _s5 = $RefreshSig$(),\n    _s6 = $RefreshSig$();\n\nfunction ownKeys(object, enumerableOnly) { var keys = Object.keys(object); if (Object.getOwnPropertySymbols) { var symbols = Object.getOwnPropertySymbols(object); if (enumerableOnly) symbols = symbols.filter(function (sym) { return Object.getOwnPropertyDescriptor(object, sym).enumerable; }); keys.push.apply(keys, symbols); } return keys; }\n\nfunction _objectSpread(target) { for (var i = 1; i < arguments.length; i++) { var source = arguments[i] != null ? arguments[i] : {}; if (i % 2) { ownKeys(Object(source), true).forEach(function (key) { Object(_home_jesse_python_swift_next_swift_node_modules_babel_runtime_helpers_esm_defineProperty__WEBPACK_IMPORTED_MODULE_1__[\"default\"])(target, key, source[key]); }); } else if (Object.getOwnPropertyDescriptors) { Object.defineProperties(target, Object.getOwnPropertyDescriptors(source)); } else { ownKeys(Object(source)).forEach(function (key) { Object.defineProperty(target, key, Object.getOwnPropertyDescriptor(source, key)); }); } } return target; }\n\n\n\n\nthree__WEBPACK_IMPORTED_MODULE_4__[\"Object3D\"].DefaultUp.set(0, 0, 1);\nvar Loader = /*#__PURE__*/Object(react__WEBPACK_IMPORTED_MODULE_2__[\"lazy\"])(_c = function _c() {\n  return __webpack_require__.e(/*! import() */ 1).then(__webpack_require__.bind(null, /*! ./Loader */ \"./components/Loader.tsx\"));\n});\n_c2 = Loader;\nvar Plane = function Plane() {\n  _s();\n\n  var _useThree = Object(react_three_fiber__WEBPACK_IMPORTED_MODULE_3__[\"useThree\"])(),\n      scene = _useThree.scene;\n\n  scene.background = new three__WEBPACK_IMPORTED_MODULE_4__[\"Color\"](0x787878);\n  scene.fog = new three__WEBPACK_IMPORTED_MODULE_4__[\"Fog\"](0x787878, 50, 60);\n  return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"mesh\", {\n    receiveShadow: true,\n    children: [/*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"planeBufferGeometry\", {\n      args: [200, 200]\n    }, void 0, false, {\n      fileName: _jsxFileName,\n      lineNumber: 21,\n      columnNumber: 13\n    }, _this), /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"meshPhongMaterial\", {\n      color: 0x4b4b4b,\n      specular: new three__WEBPACK_IMPORTED_MODULE_4__[\"Color\"](0x101010)\n    }, void 0, false, {\n      fileName: _jsxFileName,\n      lineNumber: 22,\n      columnNumber: 13\n    }, _this)]\n  }, void 0, true, {\n    fileName: _jsxFileName,\n    lineNumber: 20,\n    columnNumber: 9\n  }, _this);\n};\n\n_s(Plane, \"H/W0sI++KbHVw4gYmxbTnW2ERRY=\", false, function () {\n  return [react_three_fiber__WEBPACK_IMPORTED_MODULE_3__[\"useThree\"]];\n});\n\n_c3 = Plane;\nvar ShadowedLight = function ShadowedLight(props) {\n  _s2();\n\n  var light = Object(react__WEBPACK_IMPORTED_MODULE_2__[\"useRef\"])(); // const d = 1\n  // useEffect(() => {\n  // light.current.shadow.camera.left = -d\n  // light.current.shadow.camera.right = d\n  // light.current.shadow.camera.top = d\n  // light.current.shadow.camera.bottom = -d\n  // light.current.shadow.camera.near = 0\n  // light.current.shadow.camera.far = 40\n  // light.current.shadow.bias = -0.002\n  // }, [])\n\n  return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"directionalLight\", {\n    ref: light,\n    color: props.color,\n    intensity: props.intensity,\n    position: [props.x, props.y, props.z],\n    castShadow: true\n  }, void 0, false, {\n    fileName: _jsxFileName,\n    lineNumber: 55,\n    columnNumber: 9\n  }, _this);\n};\n\n_s2(ShadowedLight, \"FfjgNCujpMxs+tY9urTRo62RsQE=\");\n\n_c4 = ShadowedLight;\nvar Camera = function Camera(props) {\n  _s3();\n\n  var _useThree2 = Object(react_three_fiber__WEBPACK_IMPORTED_MODULE_3__[\"useThree\"])(),\n      viewport = _useThree2.viewport,\n      setDefaultCamera = _useThree2.setDefaultCamera;\n\n  var width = viewport.width,\n      height = viewport.height;\n  var camera = Object(react__WEBPACK_IMPORTED_MODULE_2__[\"useRef\"])();\n  Object(react__WEBPACK_IMPORTED_MODULE_2__[\"useEffect\"])(function () {\n    if (props.setDefault) {\n      setDefaultCamera(camera.current);\n    }\n  });\n  Object(react_three_fiber__WEBPACK_IMPORTED_MODULE_3__[\"useFrame\"])(function (state, delta) {\n    props.fpsCallBack(1.0 / delta);\n  });\n  return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"perspectiveCamera\", {\n    ref: camera,\n    position: [0.2, 1.2, 0.7],\n    near: 0.01,\n    far: 100,\n    fov: 70,\n    aspect: height / width\n  }, void 0, false, {\n    fileName: _jsxFileName,\n    lineNumber: 88,\n    columnNumber: 9\n  }, _this);\n};\n\n_s3(Camera, \"v7bmkvP7rxEF7/vR8fZc9wSYT18=\", false, function () {\n  return [react_three_fiber__WEBPACK_IMPORTED_MODULE_3__[\"useThree\"], react_three_fiber__WEBPACK_IMPORTED_MODULE_3__[\"useFrame\"]];\n});\n\n_c5 = Camera;\n\nvar PrimativeShapes = function PrimativeShapes(props) {\n  switch (props.stype) {\n    case 'box':\n      return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"boxBufferGeometry\", {\n        args: [props.scale[0], props.scale[1], props.scale[2]]\n      }, void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 103,\n        columnNumber: 17\n      }, _this);\n      break;\n\n    case 'sphere':\n      return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"sphereBufferGeometry\", {\n        args: [props.radius, 64, 64]\n      }, void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 110,\n        columnNumber: 20\n      }, _this);\n      break;\n\n    case 'cylinder':\n      return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"cylinderBufferGeometry\", {\n        args: [props.radius, props.radius, props.length, 32]\n      }, void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 115,\n        columnNumber: 17\n      }, _this);\n      break;\n\n    default:\n      return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"boxBufferGeometry\", {\n        args: [props.scale[0], props.scale[1], props.scale[2]]\n      }, void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 123,\n        columnNumber: 17\n      }, _this);\n  }\n};\n\n_c6 = PrimativeShapes;\n\nvar BasicShape = function BasicShape(props) {\n  _s4();\n\n  var shape = Object(react__WEBPACK_IMPORTED_MODULE_2__[\"useRef\"])(); // useEffect(() => {\n  //     if (props.q) {\n  //         console.log(props.q)\n  //         // const q = new THREE.Quaternion(\n  //         //     props.q[1],\n  //         //     props.q[2],\n  //         //     props.q[3],\n  //         //     props.q[0]\n  //         // )\n  //         // shape.current.setRotationFromQuaternion(q)\n  //         // console.log(shape)\n  //         // shape.current.quaternion.set(\n  //         //     props.q[1],\n  //         //     props.q[2],\n  //         //     props.q[3],\n  //         //     props.q[0]\n  //         // )\n  //     }\n  // }, [])\n  // useFrame(() => {\n  //     if (shape.current) {\n  //         console.log(shape.current.up)\n  //     } else {\n  //         console.log(shape)\n  //     }\n  //     // console.log(shape.current.DefaultUp)\n  // })\n\n  return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"mesh\", {\n    ref: shape,\n    position: new three__WEBPACK_IMPORTED_MODULE_4__[\"Vector3\"](props.t[0], props.t[1], props.t[2]),\n    quaternion: new three__WEBPACK_IMPORTED_MODULE_4__[\"Quaternion\"](props.q[0], props.q[1], props.q[2], props.q[3]),\n    castShadow: true,\n    name: 'loaded',\n    children: [/*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(PrimativeShapes, _objectSpread({}, props), void 0, false, {\n      fileName: _jsxFileName,\n      lineNumber: 191,\n      columnNumber: 13\n    }, _this), /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"meshStandardMaterial\", {\n      transparent: props.opacity ? true : false,\n      color: props.color ? props.color : 'hotpink',\n      opacity: props.opacity ? props.opacity : 1.0\n    }, void 0, false, {\n      fileName: _jsxFileName,\n      lineNumber: 192,\n      columnNumber: 13\n    }, _this)]\n  }, void 0, true, {\n    fileName: _jsxFileName,\n    lineNumber: 177,\n    columnNumber: 9\n  }, _this);\n};\n\n_s4(BasicShape, \"2xevduJYtiKmJORhM+aoFoebnMU=\");\n\n_c7 = BasicShape;\n\nvar MeshShape = function MeshShape(props) {\n  _s5();\n\n  var _useState = Object(react__WEBPACK_IMPORTED_MODULE_2__[\"useState\"])(false),\n      hasMounted = _useState[0],\n      setHasMounted = _useState[1];\n\n  Object(react__WEBPACK_IMPORTED_MODULE_2__[\"useEffect\"])(function () {\n    setHasMounted(true); // const q = new THREE.Quaternion(\n    //     props.q[1],\n    //     props.q[2],\n    //     props.q[3],\n    //     props.q[0]\n    // )\n    // shape.current.setRotationFromQuaternion(q)\n    // console.log(shape)\n  }, []);\n  return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"Fragment\"], {\n    children: hasMounted && /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(react__WEBPACK_IMPORTED_MODULE_2__[\"Suspense\"], {\n      fallback: /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(BasicShape, {\n        stype: 'box',\n        scale: [0.1, 0.1, 0.1],\n        t: props.t // q={props.q}\n        ,\n        opacity: 0.1,\n        color: 0xffffff\n      }, void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 222,\n        columnNumber: 25\n      }, _this),\n      children: /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(Loader, _objectSpread({}, props), void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 232,\n        columnNumber: 21\n      }, _this)\n    }, void 0, false, {\n      fileName: _jsxFileName,\n      lineNumber: 220,\n      columnNumber: 17\n    }, _this)\n  }, void 0, false);\n};\n\n_s5(MeshShape, \"aiSd/DQPOnbbLLZZL0Xv/KtPBDg=\");\n\n_c8 = MeshShape;\n\nvar AxesShape = function AxesShape(props) {\n  _s6();\n\n  var shape = Object(react__WEBPACK_IMPORTED_MODULE_2__[\"useRef\"])();\n  return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"mesh\", {\n    ref: shape,\n    position: new three__WEBPACK_IMPORTED_MODULE_4__[\"Vector3\"](props.t[0], props.t[1], props.t[2]),\n    quaternion: new three__WEBPACK_IMPORTED_MODULE_4__[\"Quaternion\"](props.q[0], props.q[1], props.q[2], props.q[3]),\n    name: 'loaded',\n    children: /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(\"axesHelper\", {\n      args: [props.length]\n    }, void 0, false, {\n      fileName: _jsxFileName,\n      lineNumber: 256,\n      columnNumber: 13\n    }, _this)\n  }, void 0, false, {\n    fileName: _jsxFileName,\n    lineNumber: 243,\n    columnNumber: 9\n  }, _this);\n};\n\n_s6(AxesShape, \"2xevduJYtiKmJORhM+aoFoebnMU=\");\n\n_c9 = AxesShape;\nvar Shape = function Shape(props) {\n  if (props.display === false) {\n    return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"Fragment\"], {}, void 0, false);\n  }\n\n  switch (props.stype) {\n    case 'mesh':\n      return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(MeshShape, _objectSpread({}, props), void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 268,\n        columnNumber: 20\n      }, _this);\n      break;\n\n    case 'axes':\n      return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(AxesShape, _objectSpread({}, props), void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 272,\n        columnNumber: 20\n      }, _this);\n      break;\n\n    case 'box':\n    case 'cylinder':\n    case 'sphere':\n    default:\n      return /*#__PURE__*/Object(react_jsx_dev_runtime__WEBPACK_IMPORTED_MODULE_0__[\"jsxDEV\"])(BasicShape, _objectSpread({}, props), void 0, false, {\n        fileName: _jsxFileName,\n        lineNumber: 279,\n        columnNumber: 20\n      }, _this);\n      break;\n  }\n};\n_c10 = Shape;\n\nvar _c, _c2, _c3, _c4, _c5, _c6, _c7, _c8, _c9, _c10;\n\n$RefreshReg$(_c, \"Loader$lazy\");\n$RefreshReg$(_c2, \"Loader\");\n$RefreshReg$(_c3, \"Plane\");\n$RefreshReg$(_c4, \"ShadowedLight\");\n$RefreshReg$(_c5, \"Camera\");\n$RefreshReg$(_c6, \"PrimativeShapes\");\n$RefreshReg$(_c7, \"BasicShape\");\n$RefreshReg$(_c8, \"MeshShape\");\n$RefreshReg$(_c9, \"AxesShape\");\n$RefreshReg$(_c10, \"Shape\");\n\n;\n    var _a, _b;\n    // Legacy CSS implementations will `eval` browser code in a Node.js context\n    // to extract CSS. For backwards compatibility, we need to check we're in a\n    // browser context before continuing.\n    if (typeof self !== 'undefined' &&\n        // AMP / No-JS mode does not inject these helpers:\n        '$RefreshHelpers$' in self) {\n        var currentExports = module.__proto__.exports;\n        var prevExports = (_b = (_a = module.hot.data) === null || _a === void 0 ? void 0 : _a.prevExports) !== null && _b !== void 0 ? _b : null;\n        // This cannot happen in MainTemplate because the exports mismatch between\n        // templating and execution.\n        self.$RefreshHelpers$.registerExportsForReactRefresh(currentExports, module.i);\n        // A module can be accepted automatically based on its exports, e.g. when\n        // it is a Refresh Boundary.\n        if (self.$RefreshHelpers$.isReactRefreshBoundary(currentExports)) {\n            // Save the previous exports on update so we can compare the boundary\n            // signatures.\n            module.hot.dispose(function (data) {\n                data.prevExports = currentExports;\n            });\n            // Unconditionally accept an update to this module, we'll check if it's\n            // still a Refresh Boundary later.\n            module.hot.accept();\n            // This field is set when the previous version of this module was a\n            // Refresh Boundary, letting us know we need to check for invalidation or\n            // enqueue an update.\n            if (prevExports !== null) {\n                // A boundary can become ineligible if its exports are incompatible\n                // with the previous exports.\n                //\n                // For example, if you add/remove/change exports, we'll want to\n                // re-execute the importing modules, and force those components to\n                // re-render. Similarly, if you convert a class component to a\n                // function, we want to invalidate the boundary.\n                if (self.$RefreshHelpers$.shouldInvalidateReactRefreshBoundary(prevExports, currentExports)) {\n                    module.hot.invalidate();\n                }\n                else {\n                    self.$RefreshHelpers$.scheduleUpdate();\n                }\n            }\n        }\n        else {\n            // Since we just executed the code for the module, it's possible that the\n            // new exports made it ineligible for being a boundary.\n            // We only care about the case when we were _previously_ a boundary,\n            // because we already accepted this update (accidental side effect).\n            var isNoLongerABoundary = prevExports !== null;\n            if (isNoLongerABoundary) {\n                module.hot.invalidate();\n            }\n        }\n    }\n\n/* WEBPACK VAR INJECTION */}.call(this, __webpack_require__(/*! ./../node_modules/next/dist/compiled/webpack/harmony-module.js */ \"./node_modules/next/dist/compiled/webpack/harmony-module.js\")(module)))//# sourceURL=[module]\n//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIndlYnBhY2s6Ly9fTl9FLy4vY29tcG9uZW50cy9Td2lmdENvbXBvbmVudHMudHN4PzE2YzAiXSwibmFtZXMiOlsiVEhSRUUiLCJEZWZhdWx0VXAiLCJzZXQiLCJMb2FkZXIiLCJsYXp5IiwiUGxhbmUiLCJ1c2VUaHJlZSIsInNjZW5lIiwiYmFja2dyb3VuZCIsImZvZyIsIlNoYWRvd2VkTGlnaHQiLCJwcm9wcyIsImxpZ2h0IiwidXNlUmVmIiwiY29sb3IiLCJpbnRlbnNpdHkiLCJ4IiwieSIsInoiLCJDYW1lcmEiLCJ2aWV3cG9ydCIsInNldERlZmF1bHRDYW1lcmEiLCJ3aWR0aCIsImhlaWdodCIsImNhbWVyYSIsInVzZUVmZmVjdCIsInNldERlZmF1bHQiLCJjdXJyZW50IiwidXNlRnJhbWUiLCJzdGF0ZSIsImRlbHRhIiwiZnBzQ2FsbEJhY2siLCJQcmltYXRpdmVTaGFwZXMiLCJzdHlwZSIsInNjYWxlIiwicmFkaXVzIiwibGVuZ3RoIiwiQmFzaWNTaGFwZSIsInNoYXBlIiwidCIsInEiLCJvcGFjaXR5IiwiTWVzaFNoYXBlIiwidXNlU3RhdGUiLCJoYXNNb3VudGVkIiwic2V0SGFzTW91bnRlZCIsIkF4ZXNTaGFwZSIsIlNoYXBlIiwiZGlzcGxheSJdLCJtYXBwaW5ncyI6Ijs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7QUFBQTtBQUVBO0FBQ0E7QUFNQUEsOENBQUEsQ0FBZUMsU0FBZixDQUF5QkMsR0FBekIsQ0FBNkIsQ0FBN0IsRUFBZ0MsQ0FBaEMsRUFBbUMsQ0FBbkM7QUFDQSxJQUFNQyxNQUFNLGdCQUFHQyxrREFBSSxNQUFDO0FBQUEsU0FBTSx3SEFBTjtBQUFBLENBQUQsQ0FBbkI7TUFBTUQsTTtBQUVDLElBQU1FLEtBQWUsR0FBRyxTQUFsQkEsS0FBa0IsR0FBbUI7QUFBQTs7QUFBQSxrQkFDNUJDLGtFQUFRLEVBRG9CO0FBQUEsTUFDdENDLEtBRHNDLGFBQ3RDQSxLQURzQzs7QUFHOUNBLE9BQUssQ0FBQ0MsVUFBTixHQUFtQixJ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QUFBQTtBQWJKO0FBQUE7QUFBQTtBQUFBO0FBQUEsV0FESjtBQWlCSCxDQXBCRDs7SUFBTVUsUzs7TUFBQUEsUztBQXNCQyxJQUFNQyxLQUFLLEdBQUcsU0FBUkEsS0FBUSxDQUFDcEMsS0FBRCxFQUFxQztBQUN0RCxNQUFJQSxLQUFLLENBQUNxQyxPQUFOLEtBQWtCLEtBQXRCLEVBQTZCO0FBQ3pCLHdCQUFPLHVKQUFQO0FBQ0g7O0FBRUQsVUFBUXJDLEtBQUssQ0FBQ3NCLEtBQWQ7QUFDSSxTQUFLLE1BQUw7QUFDSSwwQkFBTyxxRUFBQyxTQUFELG9CQUFldEIsS0FBZjtBQUFBO0FBQUE7QUFBQTtBQUFBLGVBQVA7QUFDQTs7QUFFSixTQUFLLE1BQUw7QUFDSSwwQkFBTyxxRUFBQyxTQUFELG9CQUFlQSxLQUFmO0FBQUE7QUFBQTtBQUFBO0FBQUEsZUFBUDtBQUNBOztBQUVKLFNBQUssS0FBTDtBQUNBLFNBQUssVUFBTDtBQUNBLFNBQUssUUFBTDtBQUNBO0FBQ0ksMEJBQU8scUVBQUMsVUFBRCxvQkFBZ0JBLEtBQWhCO0FBQUE7QUFBQTtBQUFBO0FBQUEsZUFBUDtBQUNBO0FBZFI7QUFnQkgsQ0FyQk07T0FBTW9DLEsiLCJmaWxlIjoiLi9jb21wb25lbnRzL1N3aWZ0Q29tcG9uZW50cy50c3guanMiLCJzb3VyY2VzQ29udGVudCI6WyJpbXBvcnQgUmVhY3QsIHsgdXNlRWZmZWN0LCB1c2VTdGF0ZSwgdXNlUmVmLCBTdXNwZW5zZSwgbGF6eSB9IGZyb20gJ3JlYWN0J1xuXG5pbXBvcnQgeyB1c2VUaHJlZSwgdXNlRnJhbWUgfSBmcm9tICdyZWFjdC10aHJlZS1maWJlcidcbmltcG9ydCAqIGFzIFRIUkVFIGZyb20gJ3RocmVlJ1xuaW1wb3J0IHtcbiAgICBCdWZmZXJHZW9tZXRyeSxcbiAgICBGbG9hdDMyQnVmZmVyQXR0cmlidXRlLFxuICAgIExpbmVCYXNpY01hdGVyaWFsLFxufSBmcm9tICd0aHJlZSdcblRIUkVFLk9iamVjdDNELkRlZmF1bHRVcC5zZXQoMCwgMCwgMSlcbmNvbnN0IExvYWRlciA9IGxhenkoKCkgPT4gaW1wb3J0KCcuL0xvYWRlcicpKVxuXG5leHBvcnQgY29uc3QgUGxhbmU6IFJlYWN0LkZDID0gKCk6IEpTWC5FbGVtZW50ID0+IHtcbiAgICBjb25zdCB7IHNjZW5lIH0gPSB1c2VUaHJlZSgpXG5cbiAgICBzY2VuZS5iYWNrZ3JvdW5kID0gbmV3IFRIUkVFLkNvbG9yKDB4Nzg3ODc4KVxuICAgIHNjZW5lLmZvZyA9IG5ldyBUSFJFRS5Gb2coMHg3ODc4NzgsIDUwLCA2MClcblxuICAgIHJldHVybiAoXG4gICAgICAgIDxtZXNoIHJlY2VpdmVTaGFkb3c9e3RydWV9PlxuICAgICAgICAgICAgPHBsYW5lQnVmZmVyR2VvbWV0cnkgYXJncz17WzIwMCwgMjAwXX0gLz5cbiAgICAgICAgICAgIDxtZXNoUGhvbmdNYXRlcmlhbFxuICAgICAgICAgICAgICAgIGNvbG9yPXsweDRiNGI0Yn1cbiAgICAgICAgICAgICAgICBzcGVjdWxhcj17bmV3IFRIUkVFLkNvbG9yKDB4MTAxMDEwKX1cbiAgICAgICAgICAgIC8+XG4gICAgICAgIDwvbWVzaD5cbiAgICApXG59XG5cbmV4cG9ydCBpbnRlcmZhY2UgSVNoYWRvd2VkTGlnaHRQcm9wcyB7XG4gICAgeDogbnVtYmVyXG4gICAgeTogbnVtYmVyXG4gICAgejogbnVtYmVyXG4gICAgY29sb3I6IG51bWJlclxuICAgIGludGVuc2l0eTogbnVtYmVyXG59XG5cbmV4cG9ydCBjb25zdCBTaGFkb3dlZExpZ2h0OiBSZWFjdC5GQzxJU2hhZG93ZWRMaWdodFByb3BzPiA9IChcbiAgICBwcm9wczogSVNoYWRvd2VkTGlnaHRQcm9wc1xuKTogSlNYLkVsZW1lbnQgPT4ge1xuICAgIGNvbnN0IGxpZ2h0ID0gdXNlUmVmPFRIUkVFLkRpcmVjdGlvbmFsTGlnaHQ+KClcbiAgICAvLyBjb25zdCBkID0gMVxuXG4gICAgLy8gdXNlRWZmZWN0KCgpID0+IHtcbiAgICAvLyBsaWdodC5jdXJyZW50LnNoYWRvdy5jYW1lcmEubGVmdCA9IC1kXG4gICAgLy8gbGlnaHQuY3VycmVudC5zaGFkb3cuY2FtZXJhLnJpZ2h0ID0gZFxuICAgIC8vIGxpZ2h0LmN1cnJlbnQuc2hhZG93LmNhbWVyYS50b3AgPSBkXG4gICAgLy8gbGlnaHQuY3VycmVudC5zaGFkb3cuY2FtZXJhLmJvdHRvbSA9IC1kXG4gICAgLy8gbGlnaHQuY3VycmVudC5zaGFkb3cuY2FtZXJhLm5lYXIgPSAwXG4gICAgLy8gbGlnaHQuY3VycmVudC5zaGFkb3cuY2FtZXJhLmZhciA9IDQwXG4gICAgLy8gbGlnaHQuY3VycmVudC5zaGFkb3cuYmlhcyA9IC0wLjAwMlxuICAgIC8vIH0sIFtdKVxuXG4gICAgcmV0dXJuIChcbiAgICAgICAgPGRpcmVjdGlvbmFsTGlnaHRcbiAgICAgICAgICAgIHJlZj17bGlnaHR9XG4gICAgICAgICAgICBjb2xvcj17cHJvcHMuY29sb3J9XG4gICAgICAgICAgICBpbnRlbnNpdHk9e3Byb3BzLmludGVuc2l0eX1cbiAgICAgICAgICAgIHBvc2l0aW9uPXtbcHJvcHMueCwgcHJvcHMueSwgcHJvcHMuel19XG4gICAgICAgICAgICBjYXN0U2hhZG93PXt0cnVlfVxuICAgICAgICAvPlxuICAgIClcbn1cblxuZXhwb3J0IGludGVyZmFjZSBJQ2FtZXJhUHJvcHMge1xuICAgIHNldERlZmF1bHQ6IGJvb2xlYW5cbiAgICBmcHNDYWxsQmFjazogYW55XG59XG5cbmV4cG9ydCBjb25zdCBDYW1lcmEgPSAocHJvcHM6IElDYW1lcmFQcm9wcyk6IEpTWC5FbGVtZW50ID0+IHtcbiAgICBjb25zdCB7IHZpZXdwb3J0LCBzZXREZWZhdWx0Q2FtZXJhIH0gPSB1c2VUaHJlZSgpXG5cbiAgICBjb25zdCB7IHdpZHRoLCBoZWlnaHQgfSA9IHZpZXdwb3J0XG5cbiAgICBjb25zdCBjYW1lcmEgPSB1c2VSZWY8VEhSRUUuUGVyc3BlY3RpdmVDYW1lcmE+KClcblxuICAgIHVzZUVmZmVjdCgoKSA9PiB7XG4gICAgICAgIGlmIChwcm9wcy5zZXREZWZhdWx0KSB7XG4gICAgICAgICAgICBzZXREZWZhdWx0Q2FtZXJhKGNhbWVyYS5jdXJyZW50KVxuICAgICAgICB9XG4gICAgfSlcblxuICAgIHVzZUZyYW1lKChzdGF0ZSwgZGVsdGEpID0+IHtcbiAgICAgICAgcHJvcHMuZnBzQ2FsbEJhY2soMS4wIC8gZGVsdGEpXG4gICAgfSlcblxuICAgIHJldHVybiAoXG4gICAgICAgIDxwZXJzcGVjdGl2ZUNhbWVyYVxuICAgICAgICAgICAgcmVmPXtjYW1lcmF9XG4gICAgICAgICAgICBwb3NpdGlvbj17WzAuMiwgMS4yLCAwLjddfVxuICAgICAgICAgICAgbmVhcj17MC4wMX1cbiAgICAgICAgICAgIGZhcj17MTAwfVxuICAgICAgICAgICAgZm92PXs3MH1cbiAgICAgICAgICAgIGFzcGVjdD17aGVpZ2h0IC8gd2lkdGh9XG4gICAgICAgIC8+XG4gICAgKVxufVxuXG5jb25zdCBQcmltYXRpdmVTaGFwZXMgPSAocHJvcHM6IElTaGFwZVByb3BzKTogSlNYLkVsZW1lbnQgPT4ge1xuICAgIHN3aXRjaCAocHJvcHMuc3R5cGUpIHtcbiAgICAgICAgY2FzZSAnYm94JzpcbiAgICAgICAgICAgIHJldHVybiAoXG4gICAgICAgICAgICAgICAgPGJveEJ1ZmZlckdlb21ldHJ5XG4gICAgICAgICAgICAgICAgICAgIGFyZ3M9e1twcm9wcy5zY2FsZVswXSwgcHJvcHMuc2NhbGVbMV0sIHByb3BzLnNjYWxlWzJdXX1cbiAgICAgICAgICAgICAgICAvPlxuICAgICAgICAgICAgKVxuICAgICAgICAgICAgYnJlYWtcblxuICAgICAgICBjYXNlICdzcGhlcmUnOlxuICAgICAgICAgICAgcmV0dXJuIDxzcGhlcmVCdWZmZXJHZW9tZXRyeSBhcmdzPXtbcHJvcHMucmFkaXVzLCA2NCwgNjRdfSAvPlxuICAgICAgICAgICAgYnJlYWtcblxuICAgICAgICBjYXNlICdjeWxpbmRlcic6XG4gICAgICAgICAgICByZXR1cm4gKFxuICAgICAgICAgICAgICAgIDxjeWxpbmRlckJ1ZmZlckdlb21ldHJ5XG4gICAgICAgICAgICAgICAgICAgIGFyZ3M9e1twcm9wcy5yYWRpdXMsIHByb3BzLnJhZGl1cywgcHJvcHMubGVuZ3RoLCAzMl19XG4gICAgICAgICAgICAgICAgLz5cbiAgICAgICAgICAgIClcbiAgICAgICAgICAgIGJyZWFrXG5cbiAgICAgICAgZGVmYXVsdDpcbiAgICAgICAgICAgIHJldHVybiAoXG4gICAgICAgICAgICAgICAgPGJveEJ1ZmZlckdlb21ldHJ5XG4gICAgICAgICAgICAgICAgICAgIGFyZ3M9e1twcm9wcy5zY2FsZVswXSwgcHJvcHMuc2NhbGVbMV0sIHByb3BzLnNjYWxlWzJdXX1cbiAgICAgICAgICAgICAgICAvPlxuICAgICAgICAgICAgKVxuICAgIH1cbn1cblxuZXhwb3J0IGludGVyZmFjZSBJU2hhcGVQcm9wcyB7XG4gICAgc3R5cGU6IHN0cmluZ1xuICAgIHNjYWxlPzogbnVtYmVyW11cbiAgICBmaWxlbmFtZT86IHN0cmluZ1xuICAgIHJhZGl1cz86IG51bWJlclxuICAgIGxlbmd0aD86IG51bWJlclxuICAgIHE/OiBudW1iZXJbXVxuICAgIHQ/OiBudW1iZXJbXVxuICAgIHY/OiBudW1iZXJbXVxuICAgIGNvbG9yPzogbnVtYmVyXG4gICAgb3BhY2l0eT86IG51bWJlclxuICAgIGRpc3BsYXk/OiBib29sZWFuXG59XG5cbmNvbnN0IEJhc2ljU2hhcGUgPSAocHJvcHM6IElTaGFwZVByb3BzKTogSlNYLkVsZW1lbnQgPT4ge1xuICAgIGNvbnN0IHNoYXBlID0gdXNlUmVmPFRIUkVFLk1lc2g+KClcbiAgICAvLyB1c2VFZmZlY3QoKCkgPT4ge1xuICAgIC8vICAgICBpZiAocHJvcHMucSkge1xuICAgIC8vICAgICAgICAgY29uc29sZS5sb2cocHJvcHMucSlcbiAgICAvLyAgICAgICAgIC8vIGNvbnN0IHEgPSBuZXcgVEhSRUUuUXVhdGVybmlvbihcbiAgICAvLyAgICAgICAgIC8vICAgICBwcm9wcy5xWzFdLFxuICAgIC8vICAgICAgICAgLy8gICAgIHByb3BzLnFbMl0sXG4gICAgLy8gICAgICAgICAvLyAgICAgcHJvcHMucVszXSxcbiAgICAvLyAgICAgICAgIC8vICAgICBwcm9wcy5xWzBdXG4gICAgLy8gICAgICAgICAvLyApXG4gICAgLy8gICAgICAgICAvLyBzaGFwZS5jdXJyZW50LnNldFJvdGF0aW9uRnJvbVF1YXRlcm5pb24ocSlcbiAgICAvLyAgICAgICAgIC8vIGNvbnNvbGUubG9nKHNoYXBlKVxuXG4gICAgLy8gICAgICAgICAvLyBzaGFwZS5jdXJyZW50LnF1YXRlcm5pb24uc2V0KFxuICAgIC8vICAgICAgICAgLy8gICAgIHByb3BzLnFbMV0sXG4gICAgLy8gICAgICAgICAvLyAgICAgcHJvcHMucVsyXSxcbiAgICAvLyAgICAgICAgIC8vICAgICBwcm9wcy5xWzNdLFxuICAgIC8vICAgICAgICAgLy8gICAgIHByb3BzLnFbMF1cbiAgICAvLyAgICAgICAgIC8vIClcbiAgICAvLyAgICAgfVxuICAgIC8vIH0sIFtdKVxuXG4gICAgLy8gdXNlRnJhbWUoKCkgPT4ge1xuICAgIC8vICAgICBpZiAoc2hhcGUuY3VycmVudCkge1xuICAgIC8vICAgICAgICAgY29uc29sZS5sb2coc2hhcGUuY3VycmVudC51cClcbiAgICAvLyAgICAgfSBlbHNlIHtcbiAgICAvLyAgICAgICAgIGNvbnNvbGUubG9nKHNoYXBlKVxuICAgIC8vICAgICB9XG4gICAgLy8gICAgIC8vIGNvbnNvbGUubG9nKHNoYXBlLmN1cnJlbnQuRGVmYXVsdFVwKVxuICAgIC8vIH0pXG5cbiAgICByZXR1cm4gKFxuICAgICAgICA8bWVzaFxuICAgICAgICAgICAgcmVmPXtzaGFwZX1cbiAgICAgICAgICAgIHBvc2l0aW9uPXtuZXcgVEhSRUUuVmVjdG9yMyhwcm9wcy50WzBdLCBwcm9wcy50WzFdLCBwcm9wcy50WzJdKX1cbiAgICAgICAgICAgIHF1YXRlcm5pb249e1xuICAgICAgICAgICAgICAgIG5ldyBUSFJFRS5RdWF0ZXJuaW9uKFxuICAgICAgICAgICAgICAgICAgICBwcm9wcy5xWzBdLFxuICAgICAgICAgICAgICAgICAgICBwcm9wcy5xWzFdLFxuICAgICAgICAgICAgICAgICAgICBwcm9wcy5xWzJdLFxuICAgICAgICAgICAgICAgICAgICBwcm9wcy5xWzNdXG4gICAgICAgICAgICAgICAgKVxuICAgICAgICAgICAgfVxuICAgICAgICAgICAgY2FzdFNoYWRvdz17dHJ1ZX1cbiAgICAgICAgICAgIG5hbWU9eydsb2FkZWQnfVxuICAgICAgICA+XG4gICAgICAgICAgICA8UHJpbWF0aXZlU2hhcGVzIHsuLi5wcm9wc30gLz5cbiAgICAgICAgICAgIDxtZXNoU3RhbmRhcmRNYXRlcmlhbFxuICAgICAgICAgICAgICAgIHRyYW5zcGFyZW50PXtwcm9wcy5vcGFjaXR5ID8gdHJ1ZSA6IGZhbHNlfVxuICAgICAgICAgICAgICAgIGNvbG9yPXtwcm9wcy5jb2xvciA/IHByb3BzLmNvbG9yIDogJ2hvdHBpbmsnfVxuICAgICAgICAgICAgICAgIG9wYWNpdHk9e3Byb3BzLm9wYWNpdHkgPyBwcm9wcy5vcGFjaXR5IDogMS4wfVxuICAgICAgICAgICAgLz5cbiAgICAgICAgPC9tZXNoPlxuICAgIClcbn1cblxuY29uc3QgTWVzaFNoYXBlID0gKHByb3BzOiBJU2hhcGVQcm9wcyk6IEpTWC5FbGVtZW50ID0+IHtcbiAgICBjb25zdCBbaGFzTW91bnRlZCwgc2V0SGFzTW91bnRlZF0gPSB1c2VTdGF0ZShmYWxzZSlcblxuICAgIHVzZUVmZmVjdCgoKSA9PiB7XG4gICAgICAgIHNldEhhc01vdW50ZWQodHJ1ZSlcblxuICAgICAgICAvLyBjb25zdCBxID0gbmV3IFRIUkVFLlF1YXRlcm5pb24oXG4gICAgICAgIC8vICAgICBwcm9wcy5xWzFdLFxuICAgICAgICAvLyAgICAgcHJvcHMucVsyXSxcbiAgICAgICAgLy8gICAgIHByb3BzLnFbM10sXG4gICAgICAgIC8vICAgICBwcm9wcy5xWzBdXG4gICAgICAgIC8vIClcbiAgICAgICAgLy8gc2hhcGUuY3VycmVudC5zZXRSb3RhdGlvbkZyb21RdWF0ZXJuaW9uKHEpXG4gICAgICAgIC8vIGNvbnNvbGUubG9nKHNoYXBlKVxuICAgIH0sIFtdKVxuXG4gICAgcmV0dXJuIChcbiAgICAgICAgPD5cbiAgICAgICAgICAgIHtoYXNNb3VudGVkICYmIChcbiAgICAgICAgICAgICAgICA8U3VzcGVuc2VcbiAgICAgICAgICAgICAgICAgICAgZmFsbGJhY2s9e1xuICAgICAgICAgICAgICAgICAgICAgICAgPEJhc2ljU2hhcGVcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBzdHlwZT17J2JveCd9XG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgc2NhbGU9e1swLjEsIDAuMSwgMC4xXX1cbiAgICAgICAgICAgICAgICAgICAgICAgICAgICB0PXtwcm9wcy50fVxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIC8vIHE9e3Byb3BzLnF9XG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgb3BhY2l0eT17MC4xfVxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGNvbG9yPXsweGZmZmZmZn1cbiAgICAgICAgICAgICAgICAgICAgICAgIC8+XG4gICAgICAgICAgICAgICAgICAgIH1cbiAgICAgICAgICAgICAgICA+XG4gICAgICAgICAgICAgICAgICAgIDxMb2FkZXIgey4uLnByb3BzfSAvPlxuICAgICAgICAgICAgICAgIDwvU3VzcGVuc2U+XG4gICAgICAgICAgICApfVxuICAgICAgICA8Lz5cbiAgICApXG59XG5cbmNvbnN0IEF4ZXNTaGFwZSA9IChwcm9wczogSVNoYXBlUHJvcHMpOiBKU1guRWxlbWVudCA9PiB7XG4gICAgY29uc3Qgc2hhcGUgPSB1c2VSZWY8VEhSRUUuTWVzaD4oKVxuXG4gICAgcmV0dXJuIChcbiAgICAgICAgPG1lc2hcbiAgICAgICAgICAgIHJlZj17c2hhcGV9XG4gICAgICAgICAgICBwb3NpdGlvbj17bmV3IFRIUkVFLlZlY3RvcjMocHJvcHMudFswXSwgcHJvcHMudFsxXSwgcHJvcHMudFsyXSl9XG4gICAgICAgICAgICBxdWF0ZXJuaW9uPXtcbiAgICAgICAgICAgICAgICBuZXcgVEhSRUUuUXVhdGVybmlvbihcbiAgICAgICAgICAgICAgICAgICAgcHJvcHMucVswXSxcbiAgICAgICAgICAgICAgICAgICAgcHJvcHMucVsxXSxcbiAgICAgICAgICAgICAgICAgICAgcHJvcHMucVsyXSxcbiAgICAgICAgICAgICAgICAgICAgcHJvcHMucVszXVxuICAgICAgICAgICAgICAgIClcbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIG5hbWU9eydsb2FkZWQnfVxuICAgICAgICA+XG4gICAgICAgICAgICA8YXhlc0hlbHBlciBhcmdzPXtbcHJvcHMubGVuZ3RoXX0gLz5cbiAgICAgICAgPC9tZXNoPlxuICAgIClcbn1cblxuZXhwb3J0IGNvbnN0IFNoYXBlID0gKHByb3BzOiBJU2hhcGVQcm9wcyk6IEpTWC5FbGVtZW50ID0+IHtcbiAgICBpZiAocHJvcHMuZGlzcGxheSA9PT0gZmFsc2UpIHtcbiAgICAgICAgcmV0dXJuIDw+PC8+XG4gICAgfVxuXG4gICAgc3dpdGNoIChwcm9wcy5zdHlwZSkge1xuICAgICAgICBjYXNlICdtZXNoJzpcbiAgICAgICAgICAgIHJldHVybiA8TWVzaFNoYXBlIHsuLi5wcm9wc30gLz5cbiAgICAgICAgICAgIGJyZWFrXG5cbiAgICAgICAgY2FzZSAnYXhlcyc6XG4gICAgICAgICAgICByZXR1cm4gPEF4ZXNTaGFwZSB7Li4ucHJvcHN9IC8+XG4gICAgICAgICAgICBicmVha1xuXG4gICAgICAgIGNhc2UgJ2JveCc6XG4gICAgICAgIGNhc2UgJ2N5bGluZGVyJzpcbiAgICAgICAgY2FzZSAnc3BoZXJlJzpcbiAgICAgICAgZGVmYXVsdDpcbiAgICAgICAgICAgIHJldHVybiA8QmFzaWNTaGFwZSB7Li4ucHJvcHN9IC8+XG4gICAgICAgICAgICBicmVha1xuICAgIH1cbn1cbiJdLCJzb3VyY2VSb290IjoiIn0=\n//# sourceURL=webpack-internal:///./components/SwiftComponents.tsx\n");

/***/ })

})