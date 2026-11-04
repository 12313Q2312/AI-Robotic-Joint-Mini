# CAD测试资产

原生T001：`cad_native/import_test/T001_test_link_r01.SLDPRT`。名义板尺寸180×30×8 mm，两贯穿孔直径12/6 mm，中心距150 mm。它用于导入/几何检查，不是最终承载连杆。

`import_fixture/base_link.urdf` 是合成两刚体检查夹具，不宣称来自T001的真实CAD导出。名义参考图位于 `../output/pdf/`，不带制造公差，也不是原生SLDDRW。材料、惯量、装配和实际导入必须另外验证。
