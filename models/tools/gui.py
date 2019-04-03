#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PyQt5 import QtWidgets, QtCore, QtGui
from generator import *
from code_generator import CodeGenerator

class NonLinearBox(QtWidgets.QWidget) :
  def __init__(self, parent=None) :
    QtWidgets.QWidget.__init__(self, parent)
    self.nonlinear_layout = QtWidgets.QHBoxLayout()
    self.items = {
      'sbox' : QtWidgets.QSpinBox(),
      'trig' : QtWidgets.QComboBox(),
      'var' : QtWidgets.QComboBox(),
    }
    for key in ['', 'sin', 'cos'] :
      self.items['trig'].addItem(key)
    for key in ['sbox', 'trig', 'var'] :
      self.nonlinear_layout.addWidget(self.items[key])
    self.setLayout(self.nonlinear_layout)
    self.layout().setSpacing(0)
    self.layout().setContentsMargins(0,0,5,5)

class ModelGenGUI(object) :
  def __init__(self, **kwargs) :
    self.dim = 2
    self.u_dim = 1

    self.widgets = QtWidgets.QWidget()
    # text edit widget for source codes
    self.text = QtWidgets.QTextEdit()
    self.text_main = QtWidgets.QTextEdit()
    self.text_test = QtWidgets.QTextEdit()
    self.text_latex = QtWidgets.QTextEdit()
    self.text_group = QtWidgets.QGroupBox("output")
    self.text_layout = QtWidgets.QGridLayout()

    # prepare button to save generated code
    self.save_model_btn = QtWidgets.QPushButton("save model")
    self.save_main_btn = QtWidgets.QPushButton("save main")
    self.save_test_btn = QtWidgets.QPushButton("save test")
    ## button to save .tex and .pdf
    self.save_latex_btn = QtWidgets.QPushButton("save latex")
    self.save_pdf_btn = QtWidgets.QPushButton("save pdf")
    btn = [self.save_model_btn, self.save_main_btn, self.save_test_btn]
    tex_btn = [self.save_latex_btn, self.save_pdf_btn]

    # prepare tab page
    self.page_model = QtWidgets.QGridLayout()
    self.page_main = QtWidgets.QGridLayout()
    self.page_test = QtWidgets.QGridLayout()
    self.page_latex = QtWidgets.QGridLayout()
    page = [self.page_model, self.page_main, self.page_test, self.page_latex]
    ## add text and button to the page
    self.page_model.addWidget(self.text,0,0)
    self.page_main.addWidget(self.text_main,0,0)
    self.page_test.addWidget(self.text_test,0,0)
    self.page_latex.addWidget(self.text_latex,0,0)
    hlayout, widget = QtWidgets.QHBoxLayout, QtWidgets.QWidget
    ### prepare nice layout
    bl = [hlayout(), hlayout(), hlayout(), hlayout()]
    ### dummy widget for spacer
    dw = [widget(), widget(), widget(), widget()]
    for i in range(len(bl)) :
      if i >= len(btn) :
        continue
      bl[i].addWidget(dw[i])
      bl[i].addWidget(btn[i])
      page[i].addLayout(bl[i],1,0)
    ### add two buttons for latex page
    bl[3].addWidget(dw[3])
    for b in tex_btn :
      bl[3].addWidget(b)
    page[3].addLayout(bl[3],1,0)

    # tab settings for source codes
    self.tab = QtWidgets.QTabWidget()
    # self.tab.insertTab(0,self.text, "models")
    # self.tab.insertTab(1,self.text_main, "main")
    # self.tab.insertTab(2,self.text_test, "test")
    label = ["models", "main", "test", "latex"]
    for i in range(len(label)) :
      w = widget()
      w.setLayout(page[i])
      self.tab.insertTab(i,w,label[i])

    self.text_layout.addWidget(self.tab,0,0)
    self.text_group.setLayout(self.text_layout)

    # prepare layout
    self.cfg_layout = QtWidgets.QGridLayout()
    self.main_layout = QtWidgets.QGridLayout()
    self.matrix_layout = {
      'page' : {
        'linear' : QtWidgets.QVBoxLayout(),
        'nonlinear' : QtWidgets.QVBoxLayout()
      },
      'system' : {
        'linear' : QtWidgets.QGridLayout(),
        'nonlinear' : QtWidgets.QGridLayout()
      },
      'input' : {
        'linear' : QtWidgets.QGridLayout(),
        'nonlinear' : QtWidgets.QGridLayout()
      }
    }
    # prepare model group
    self.model_tab = QtWidgets.QTabWidget()

    self.main_layout.addWidget(self.text_group,0,0)

    self.dim_box = QtWidgets.QSpinBox()
    self.u_dim_box = QtWidgets.QSpinBox()
    self.model_name_layout = QtWidgets.QGridLayout()
    self.model_name = QtWidgets.QLineEdit()
    self.model_name.setMaximumWidth(200)

    # set layout for nice model name setting
    model_label = QtWidgets.QLabel('Model Name ')
    model_label.setMaximumWidth(100)
    self.model_name_layout.addWidget(model_label,0,0)
    self.model_name_layout.addWidget(self.model_name,0,1)
    self.cfg_layout.addLayout(self.model_name_layout, 0, 0)

    # prepare layout for nice dimension config spinbox
    dim_groupbox = QtWidgets.QGroupBox("dimension")
    dim_layout = QtWidgets.QGridLayout()
    dim_layout.addWidget(QtWidgets.QLabel('state dimension : '), 0, 0)
    dim_layout.addWidget(self.dim_box, 0, 1)
    dim_layout.addWidget(QtWidgets.QLabel('input dimension : '), 1, 0)
    dim_layout.addWidget(self.u_dim_box, 1, 1)
    dim_groupbox.setLayout(dim_layout)
    self.cfg_layout.addWidget(dim_groupbox, 1, 0)

    # set layout for model configuration
    model_page_label = ["linear", "nonlinear"]
    for model_label in model_page_label :
      w = QtWidgets.QWidget()
      w.setLayout(self.matrix_layout['page'][model_label])
      self.model_tab.addTab(w, model_label)
    # add model tab to config layout
    self.cfg_layout.addWidget(self.model_tab, 2, 0)

    self.main_layout.addLayout(self.cfg_layout, 0, 1)
    self.widgets.setLayout(self.main_layout)

    # set layout for the state space
    self.sboxes = {
      'linear' : {
        'system' : [], 'input' : []
      },
      'nonlinear' : {
        'system' : [], 'input' : []
      }
    }
    self.matrix_group = {
      'linear' : {
        'system' : QtWidgets.QGroupBox("Dynamic Matrix (A)"),
        'input' : QtWidgets.QGroupBox("Input Matrix (B)"),
      },
      'nonlinear' : {
        'system' : QtWidgets.QGroupBox("Dynamic Matrix (A); f(x,u) = A(x) + Bu"),
        'input' : QtWidgets.QGroupBox("Input Matrix (B)"),
        'area' : {
          'system' : QtWidgets.QScrollArea(), 
          'input' : QtWidgets.QScrollArea(),
        }
      },
    }

    scrollarea = self.matrix_group['nonlinear']['area']
    for key in ['system', 'input'] :
      group = self.matrix_group['nonlinear']
      scrollarea[key].setWidget(group[key])
      scrollarea[key].setWidgetResizable(True)
    
    for key in ['linear', 'nonlinear'] :
      for kkey in ['system', 'input'] :
        widget = self.matrix_group[key] if key == 'linear' else self.matrix_group[key]['area']
        self.matrix_layout['page'][key].addWidget(widget[kkey])

    # self.sboxes = []
    # self.dynamic_group = QtWidgets.QGroupBox("Dynamic Matrix (A)")
    # self.dynamic_group_layout = QtWidgets.QGridLayout()
    # self.u_sboxes = []
    # self.input_group = QtWidgets.QGroupBox("Input Matrix (B)")
    # self.input_group_layout = QtWidgets.QGridLayout()

    # self.sboxes = self.boxes['linear']['system']
    # self.u_sboxes = self.boxes['linear']['input']
    # self.dynamic_group = self.matrix_group['linear']['system']
    # self.dynamic_group_layout = self.matrix_layout['linear']
    # self.input_group = self.matrix_group['linear']['input']
    # self.input_group_layout = self.matrix_layout['linear']

    # for page_label in model_page_label :
    #   w = widget()
    #   self.model_tab.addTab()

    self.dim_box.setValue(self.dim)
    self.u_dim_box.setValue(self.u_dim)

    self.setSysLayout()

    self.gen_btn = QtWidgets.QPushButton("generate!")
    self.gen_btn.clicked.connect(self.generate)
    self.cfg_layout.addWidget(self.gen_btn,3,0)
    # self.cfg_layout.addWidget(self.save_model_btn,5,0)
    # self.cfg_layout.addWidget(self.save_main_btn,6,0)

    # connect widget
    ## save button
    self.save_model_btn.clicked.connect(self.save_model)
    self.save_main_btn.clicked.connect(self.save_main)
    self.save_test_btn.clicked.connect(self.save_test)
    self.save_latex_btn.clicked.connect(self.save_latex)
    self.save_pdf_btn.clicked.connect(self.save_pdf)
    ## spinbox state and input dimension
    self.dim_box.valueChanged.connect(self.setSysLayout)
    self.u_dim_box.valueChanged.connect(self.setSysLayout)

    ## code generator instance
    self.code_gen = CodeGenerator()

  def save_text(self, text) :
    filename = QtWidgets.QFileDialog.getSaveFileName()
    src = text
    print filename
    if filename[0] :
      with open(filename[0],'w+') as f:
	      f.write(src)

  def save_model(self) :
    src = self.text.toPlainText()
    self.save_text(src)

  def save_main(self) :
    src = self.text_main.toPlainText()
    self.save_text(src)
				
  def save_test(self) :
    src = self.text_test.toPlainText()
    self.save_text(src)

  def save_latex(self) :
    src = self.text_latex.toPlainText()
    self.save_text(src)
  
  def save_pdf(self) :
    filename = QtWidgets.QFileDialog.getSaveFileName()
    print filename
    if filename[0] :
      self.code_gen.generate_pdf(filename[0])

  def generate(self) :
    A, B = [], []
    nA, nB, trig, var = [], [], [], []
    for i in range(self.dim) :
      av, bv = [], []
      n_av, n_bv, n_tv, n_vv = [], [], [], []
      for b in range(self.u_dim) :
        u_sboxes = self.sboxes['linear']['input']
        bv.append(u_sboxes[i*self.u_dim+b].value())
      for c in range(self.u_dim) :
        u_sboxes = self.sboxes['nonlinear']['input']
        n_bv.append(u_sboxes[i*self.u_dim+c].value())
      for j in range(self.dim) :
        sboxes = self.sboxes['linear']['system']
        av.append(sboxes[i*self.dim+j].value())
      for k in range(self.dim) :
        sboxes = self.sboxes['nonlinear']['system'][i*self.dim+k]
        items = sboxes.items
        n_av.append(items['sbox'].value())
        n_tv.append(items['trig'].currentText())
        n_vv.append(items['var'].currentText())
      A.append(av), B.append(bv)
      nA.append(n_av), nB.append(n_bv)
      trig.append(n_tv), var.append(n_vv)
    
    print nA, nB, trig, var

    # test
    self.code_gen.generate_nonlinear_controller(nA, nB, trig, var)

    dim, u_dim = self.dim, self.u_dim
    model_name = self.model_name.text()

    def html_color(str, color) :
      return '<span style="color:%s;">%s</span>' %(color,str)
    def replace_color(str, key, color) :
      return str.replace(key, html_color(key,color))
    def parse_cpp (str) :
      kwords = {
        '#ifndef ' : 'violet',
        '#define ' : 'violet',
        '#include ' : 'violet',
        'namespace ' : 'blue',
        'const ' : 'blue',
        'int ' : 'blue',
        'typedef ' : 'blue',
        'double ' : 'blue',
        'struct ' : 'blue',
        'operator' : 'violet',
        'return ' : 'violet',
        'auto ' : 'blue',
        '#endif ' : 'violet',
        'StateSpaceSolver' : 'brown',
        'StateSpace' : 'brown',
        'LQRSolver' : 'brown',
        'FeedbackController' : 'brown',
        'FixedTimeLQR' : 'brown',
        'OptimalTimeFinder' : 'brown',
        'OptTrjSolver' : 'brown',
        '%sClosedExpm'%model_name : 'magenta',
        '%sSSComposite'%model_name : 'magenta',
        '%sSS'%model_name : 'magenta',
        '%sSolver'%model_name : 'magenta',
        '%sLQRControl'%model_name : 'magenta',
        '%sLQR'%model_name : 'magenta',
        '%sJordanForm'%model_name : 'magenta',
        '%sTimeOptDiff'%model_name : 'magenta',
        '%sGramian'%model_name : 'magenta',
        '%sFixTimeLQR'%model_name : 'magenta',
        '%sOptTimeSolver'%model_name : 'magenta',
        '%sTrajectorySolver'%model_name : 'magenta'
      }
      str = str.replace('<','&#60;').replace('>','&#62;').replace('\n','<br>')
      for k, v in kwords.items() :
        str = replace_color(str,k,v)
      return str

    # [g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str, cmp_exp] = generate_controller(A,B)
    # code_str = generate_cpp(model_name, dim, u_dim, g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str, cmp_exp)
    code_gen = self.code_gen
    code_gen.generate_controller(A,B)
    code_str = code_gen.generate_ccode(model_name, dim, u_dim)
    code_src = generate_cpp_main(model_name)
    code_test = code_gen.generate_test_ccode(model_name, dim, u_dim)
    code_latex = code_gen.generate_latex(model_name)
    # code_test = generate_test_cpp(model_name, dim, u_dim, g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str, cmp_exp)

    code_str = parse_cpp(code_str)
    code_src = parse_cpp(code_src)
    code_test = parse_cpp(code_test)

    self.text.append(code_str)
    self.text_main.append(code_src)
    self.text_test.append(code_test)
    self.text_latex.append(code_latex)

  def show(self) :
    self.widgets.showMaximized()

  def setSysLayout(self) :
    self.dim = self.dim_box.value()
    self.u_dim = self.u_dim_box.value()
    self.set_linear_sys_layout()
    self.set_nonlinear_sys_layout()

  def set_linear_sys_layout(self) :
    sboxes = self.sboxes['linear']['system']
    u_sboxes = self.sboxes['linear']['input']
    dynamic_group = self.matrix_group['linear']['system']
    input_group = self.matrix_group['linear']['input']
    dynamic_group_layout = self.matrix_layout['system']['linear']
    input_group_layout = self.matrix_layout['input']['linear']

    _delete_sboxes, _add = ModelGenGUI._delete_sboxes, ModelGenGUI._add

    _delete_sboxes(sboxes, dynamic_group_layout)
    _delete_sboxes(u_sboxes, input_group_layout)

    SpinBox = QtWidgets.QSpinBox
    _add(sboxes, self.dim, self.dim, SpinBox, dynamic_group_layout, dynamic_group)
    _add(u_sboxes, self.dim, self.u_dim, SpinBox, input_group_layout, input_group)

  def set_nonlinear_sys_layout(self) :
    sboxes = self.sboxes['nonlinear']['system']
    u_sboxes = self.sboxes['nonlinear']['input']
    dynamic_group = self.matrix_group['nonlinear']['system']
    input_group = self.matrix_group['nonlinear']['input']
    dynamic_group_layout = self.matrix_layout['system']['nonlinear']
    input_group_layout = self.matrix_layout['input']['nonlinear']

    _delete_sboxes, _add = ModelGenGUI._delete_sboxes, ModelGenGUI._add

    _delete_sboxes(sboxes, dynamic_group_layout)
    _delete_sboxes(u_sboxes, input_group_layout)

    SpinBox = NonLinearBox
    _add(sboxes, self.dim, self.dim, SpinBox, dynamic_group_layout, dynamic_group)
    SpinBox = QtWidgets.QSpinBox
    _add(u_sboxes, self.dim, self.u_dim, SpinBox, input_group_layout, input_group)

    states = ['x{}'.format(i) for i in range(self.dim)]
    states = [''] + states
    for i in range(self.dim) :
      for j in range(self.dim) :
        sboxes[i*self.dim+j].setToolTip('{},{}'.format(i+1,j+1))
        for state in states :
          sboxes[i*self.dim+j].items['var'].addItem(state)
          
  @staticmethod
  def _add(sboxes, x_dim, y_dim, obj, layout, group) :
    layout.setContentsMargins(0,0,0,0)
    layout.setSpacing(0)
    layout.setVerticalSpacing(0)
    layout.setHorizontalSpacing(0)
    for i in range(x_dim) :
      for j in range(y_dim) :
        sboxes.append(obj()) 
        layout.addWidget(sboxes[-1],i,j)
    group.setLayout(layout)

  @staticmethod
  def _delete_sboxes(sboxes, group_layout) :
    if len(sboxes) :
      for s in sboxes :
        s.deleteLater()
        group_layout.removeWidget(s)
      del sboxes[:]