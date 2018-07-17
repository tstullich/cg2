#include "SidebarWidget.hpp"

const QString SidebarWidget::CURRENT_VALUE_LABEL = "Current Value: ";

SidebarWidget::SidebarWidget(QWidget *parent) : QDockWidget(parent) {
  setWindowTitle("Settings");
  setAllowedAreas(Qt::RightDockWidgetArea);

  container = new QWidget();
  layout = new QVBoxLayout();

  initMeshBox(parent);
  initNormalCalculationBox(parent);
  initGraphLaplaceBox(parent);
  initCotanLaplaceBox(parent);

  // Mesh Section
  auto meshBox = new QGroupBox("Initial Mesh");
  auto meshLayout = new QVBoxLayout;
  meshLayout->addWidget(drawMeshBox);

  auto meshAlphaLayout = new QHBoxLayout;
  meshAlphaLayout->addWidget(new QLabel("Alpha:"));
  meshAlphaLayout->addWidget(meshAlphaBox);
  auto meshAlphaWrapper = new QWidget;
  meshAlphaWrapper->setLayout(meshAlphaLayout);
  meshLayout->addWidget(meshAlphaWrapper);
  meshBox->setLayout(meshLayout);

  // Normal Calculation Section
  auto normalBox = new QGroupBox("Normal Calculation");
  auto normalBoxLayout = new QVBoxLayout;
  normalBoxLayout->addWidget(drawUnweightedNormalsBox);
  normalBoxLayout->addWidget(drawWeightedNormalsBox);
  normalBox->setLayout(normalBoxLayout);

  // Graph Laplace Section
  auto graphLaplaceBox = new QGroupBox("Graph Laplace");
  auto graphLaplaceLayout = new QVBoxLayout;
  graphLaplaceLayout->addWidget(drawGraphLaplaceBox);

  auto stepSizeLayout = new QHBoxLayout;
  stepSizeLayout->addWidget(new QLabel("Step Size:"));
  stepSizeLayout->addWidget(stepSizeBox);
  auto stepSizeWrapper = new QWidget;
  stepSizeWrapper->setLayout(stepSizeLayout);
  graphLaplaceLayout->addWidget(stepSizeWrapper);

  graphLaplaceLayout->addWidget(graphLaplaceMoveButton);
  graphLaplaceLayout->addWidget(graphLaplaceResetButton);
  graphLaplaceBox->setLayout(graphLaplaceLayout);

  // Cotan Laplace Section
  auto cotanLaplaceBox = new QGroupBox("Cotan Laplace");
  auto cotanLaplaceLayout = new QVBoxLayout;
  cotanLaplaceLayout->addWidget(drawCotanLaplaceBox);

  auto explicitLayout = new QHBoxLayout;
  explicitLayout->addWidget(new QLabel("Explicit h:"));
  explicitLayout->addWidget(explicitStepBox);
  auto explicitStepWrapper = new QWidget;
  explicitStepWrapper->setLayout(explicitLayout);
  cotanLaplaceLayout->addWidget(explicitStepWrapper);

  cotanLaplaceLayout->addWidget(explicitStepButton);

  auto implicitLayout = new QHBoxLayout;
  implicitLayout->addWidget(new QLabel("Implicit h:"));
  implicitLayout->addWidget(implicitStepBox);
  auto implicitStepWrapper = new QWidget;
  implicitStepWrapper->setLayout(implicitLayout);
  cotanLaplaceLayout->addWidget(implicitStepWrapper);

  cotanLaplaceLayout->addWidget(implicitStepButton);

  auto basisFunctionsLayout = new QHBoxLayout;
  basisFunctionsLayout->addWidget(new QLabel("Basis Functions:"));
  basisFunctionsLayout->addWidget(basisFunctionsBox);
  auto basisFunctionsWrapper = new QWidget;
  basisFunctionsWrapper->setLayout(basisFunctionsLayout);
  cotanLaplaceLayout->addWidget(basisFunctionsWrapper);

  cotanLaplaceLayout->addWidget(basisFunctionsBox);
  cotanLaplaceLayout->addWidget(manifoldHarmonicsBox);
  cotanLaplaceLayout->addWidget(cotanLaplaceResetButton);
  cotanLaplaceBox->setLayout(cotanLaplaceLayout);

  // Add all the widgets to the final layout
  layout->addWidget(meshBox);
  layout->addWidget(normalBox);
  layout->addWidget(graphLaplaceBox);
  layout->addWidget(cotanLaplaceBox);
  layout->setSizeConstraint(QLayout::SetFixedSize);

  container->setLayout(layout);
  setWidget(container);
}

void SidebarWidget::initMeshBox(QWidget *parent) {
  drawMeshBox = new QCheckBox("Draw Mesh", this);
  drawMeshBox->setCheckState(Qt::Checked);
  connect(drawMeshBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawMesh(bool)));

  meshAlphaBox = new QDoubleSpinBox(this);
  meshAlphaBox->setRange(0.1, 1.0);
  meshAlphaBox->setValue(0.5);
  meshAlphaBox->setSingleStep(0.1);
  connect(meshAlphaBox, SIGNAL(valueChanged(double)), parent,
          SLOT(setMeshAlpha(double)));
}

void SidebarWidget::initNormalCalculationBox(QWidget *parent) {
  drawUnweightedNormalsBox = new QCheckBox("Unweighted", this);
  connect(drawUnweightedNormalsBox, SIGNAL(toggled(bool)), parent,
          SLOT(setDrawUnweightedNormals(bool)));

  drawWeightedNormalsBox = new QCheckBox("Weighted", this);
  connect(drawWeightedNormalsBox, SIGNAL(toggled(bool)), parent,
          SLOT(setDrawWeightedNormals(bool)));
}

void SidebarWidget::initGraphLaplaceBox(QWidget *parent) {
  drawGraphLaplaceBox = new QCheckBox("Render", this);
  connect(drawGraphLaplaceBox, SIGNAL(toggled(bool)), parent,
          SLOT(setDrawGraphLaplace(bool)));

  stepSizeBox = new QDoubleSpinBox(this);
  stepSizeBox->setRange(0.0001, 5.0);
  stepSizeBox->setValue(0.1);
  stepSizeBox->setSingleStep(0.1);
  connect(stepSizeBox, SIGNAL(valueChanged(double)), parent,
          SLOT(setStepSize(double)));

  graphLaplaceMoveButton = new QPushButton("Move", this);
  connect(graphLaplaceMoveButton, SIGNAL(clicked()), parent,
          SLOT(graphLaplaceMove()));

  graphLaplaceResetButton = new QPushButton("Reset", this);
  connect(graphLaplaceResetButton, SIGNAL(clicked()), parent,
          SLOT(graphLaplaceReset()));
}

void SidebarWidget::initCotanLaplaceBox(QWidget *parent) {
  drawCotanLaplaceBox = new QCheckBox("Render", this);
  connect(drawCotanLaplaceBox, SIGNAL(toggled(bool)), parent,
          SLOT(setDrawCotanLaplace(bool)));

  explicitStepBox = new QDoubleSpinBox(this);
  explicitStepBox->setRange(0.001, 5.0);
  explicitStepBox->setValue(0.01);
  explicitStepBox->setSingleStep(0.01);
  connect(explicitStepBox, SIGNAL(valueChanged(double)), parent,
          SLOT(setExplicitStep(double)));

  explicitStepButton = new QPushButton("Explicit Step", this);
  connect(explicitStepButton, SIGNAL(clicked()), parent,
          SLOT(cotanLaplaceExplicitStep()));

  implicitStepBox = new QDoubleSpinBox(this);
  implicitStepBox->setRange(0.001, 5.0);
  implicitStepBox->setValue(0.01);
  implicitStepBox->setSingleStep(0.01);
  connect(implicitStepBox, SIGNAL(valueChanged(double)), parent,
          SLOT(setImplicitStep(double)));

  implicitStepButton = new QPushButton("Implicit Step", this);
  connect(implicitStepButton, SIGNAL(clicked()), parent,
          SLOT(cotanLaplaceImplicitStep()));

  basisFunctionsBox = new QSpinBox(this);
  basisFunctionsBox->setRange(2, 100);
  basisFunctionsBox->setValue(10);
  connect(basisFunctionsBox, SIGNAL(valueChanged(int)), parent,
          SLOT(setBasisFunctions(int)));

  manifoldHarmonicsBox = new QCheckBox("Manifold Harmonics", this);
  connect(manifoldHarmonicsBox, SIGNAL(toggled(bool)), parent,
          SLOT(setManifoldHarmonics(bool)));

  cotanLaplaceResetButton = new QPushButton("Reset", this);
  connect(cotanLaplaceResetButton, SIGNAL(clicked()), parent,
          SLOT(cotanLaplaceReset()));
}
