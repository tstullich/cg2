#include "SidebarWidget.hpp"

const QString SidebarWidget::CURRENT_VALUE_LABEL = "Current Value: ";

SidebarWidget::SidebarWidget(QWidget* parent) : QDockWidget(parent) {
  setWindowTitle("Settings");
  setAllowedAreas(Qt::RightDockWidgetArea);

  container = new QWidget();
  layout = new QVBoxLayout();

  initDrawPointsBox(parent);
  initGridBox(parent);
  initMarchingCubesBox(parent);

  // Points Section
  auto pointsBox = new QGroupBox("Points");
  auto pointsLayout = new QVBoxLayout;
  pointsLayout->addWidget(drawPointsBox);
  pointsLayout->addWidget(drawNormalsBox);
  pointsLayout->addWidget(flipNormalsButton);
  pointsBox->setLayout(pointsLayout);

  // Grid Section
  auto gridBox = new QGroupBox("Grid");
  auto gridBoxLayout = new QVBoxLayout;
  gridBoxLayout->addWidget(drawPositiveSamplesBox);
  gridBoxLayout->addWidget(drawNegativeSamplesBox);
  gridBoxLayout->addWidget(drawConstraintsBox);

  auto gridSubdivisionLayout = new QHBoxLayout;
  gridSubdivisionLayout->addWidget(new QLabel("Grid subdivision:"));
  gridSubdivisionLayout->addWidget(gridSubdivisionBox);

  auto gridSubdivisionWrapper = new QWidget;
  gridSubdivisionWrapper->setLayout(gridSubdivisionLayout);

  auto boundingBoxFactorLayout = new QHBoxLayout;
  boundingBoxFactorLayout->addWidget(new QLabel("Bounding box factor:"));
  boundingBoxFactorLayout->addWidget(boundingBoxFactorBox);

  auto boundingBoxFactorWrapper = new QWidget;
  boundingBoxFactorWrapper->setLayout(boundingBoxFactorLayout);

  auto epsilonLayout = new QHBoxLayout;
  epsilonLayout->addWidget(new QLabel("Epsilon:"));
  epsilonLayout->addWidget(epsilonBox);

  auto epsilonWrapper = new QWidget;
  epsilonWrapper->setLayout(epsilonLayout);

  auto radiusBoxLayout = new QHBoxLayout;
  radiusBoxLayout->addWidget(new QLabel("Radius:"));
  radiusBoxLayout->addWidget(radiusBox);

  auto radiusWrapper = new QWidget;
  radiusWrapper->setLayout(radiusBoxLayout);

  gridBoxLayout->addWidget(gridSubdivisionWrapper);
  gridBoxLayout->addWidget(boundingBoxFactorWrapper);
  gridBoxLayout->addWidget(epsilonWrapper);
  gridBoxLayout->addWidget(radiusWrapper);
  gridBoxLayout->addWidget(computeSamplesButton);
  gridBox->setLayout(gridBoxLayout);

  // Marching Cubes Section
  auto drawMCBox = new QGroupBox("Marching Cubes");
  auto drawMCLayout = new QVBoxLayout;
  drawMCLayout->addWidget(drawMCMeshBox);
  drawMCLayout->addWidget(computeMCButton);
  drawMCLayout->addWidget(computeEMCButton);
  drawMCBox->setLayout(drawMCLayout);

  // Add all the widgets to the final layout
  layout->addWidget(pointsBox);
  layout->addWidget(gridBox);
  layout->addWidget(drawMCBox);
  layout->setSizeConstraint(QLayout::SetFixedSize);
  std::cout << "Stuff" << std::endl;

  container->setLayout(layout);
  setWidget(container);
}

void SidebarWidget::initDrawPointsBox(QWidget *parent) {
  drawPointsBox = new QCheckBox("Draw Points", this);
  drawPointsBox->setCheckState(Qt::Checked);
  connect(drawPointsBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawPoints(bool)));

  drawNormalsBox = new QCheckBox("Draw Normals", this);
  drawNormalsBox->setCheckState(Qt::Checked);
  connect(drawNormalsBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawNormals(bool)));

  flipNormalsButton = new QPushButton("Flip Normals", this);
  connect(flipNormalsButton, SIGNAL(clicked()), parent, SLOT(flipNormals()));
}

void SidebarWidget::initGridBox(QWidget *parent) {
  drawPositiveSamplesBox = new QCheckBox("Draw positive samples", this);
  //drawPositiveSamplesBox->setCheckState(Qt::Checked);
  connect(drawPositiveSamplesBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawPositiveSamples(bool)));

  drawNegativeSamplesBox = new QCheckBox("Draw negative samples", this);
  //drawNegativeSamplesBox->setCheckState(Qt::Checked);
  connect(drawNegativeSamplesBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawNegativeSamples(bool)));

  drawConstraintsBox = new QCheckBox("Draw constraints", this);
  connect(drawConstraintsBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawConstraints(bool)));

  gridSubdivisionBox = new QSpinBox(this);
  gridSubdivisionBox->setRange(1, 1000);
  gridSubdivisionBox->setValue(10);
  connect(gridSubdivisionBox, SIGNAL(valueChanged(int)), parent, SLOT(setGridSubdivision(int)));

  boundingBoxFactorBox = new QDoubleSpinBox(this);
  boundingBoxFactorBox->setRange(0.0, 10.0);
  boundingBoxFactorBox->setSingleStep(0.1);
  boundingBoxFactorBox->setValue(0.5);
  connect(boundingBoxFactorBox, SIGNAL(valueChanged(double)), parent, SLOT(setBoundingBoxFactor(double)));

  epsilonBox = new QDoubleSpinBox(this);
  epsilonBox->setRange(0.0, 10.0);
  epsilonBox->setSingleStep(0.1);
  epsilonBox->setValue(0.5);
  connect(epsilonBox, SIGNAL(valueChanged(double)), parent, SLOT(setEpsilon(double)));

  radiusBox = new QDoubleSpinBox(this);
  radiusBox->setRange(0.0, 1000.0);
  radiusBox->setSingleStep(0.02);
  radiusBox->setValue(0.02);
  connect(radiusBox, SIGNAL(valueChanged(double)), parent, SLOT(setRadius(double)));

  computeSamplesButton = new QPushButton("Compute Samples", this);
  connect(computeSamplesButton, SIGNAL(clicked()), parent, SLOT(computeSamples()));
}

void SidebarWidget::initMarchingCubesBox(QWidget *parent) {
  drawMCMeshBox = new QCheckBox("Draw MC mesh", this);
  connect(drawMCMeshBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawMCMesh(bool)));

  computeMCButton = new QPushButton("Compute MC", this);
  connect(computeMCButton, SIGNAL(clicked()), parent, SLOT(computeMC()));

  computeEMCButton = new QPushButton("Compute EMC", this);
  connect(computeEMCButton, SIGNAL(clicked()), parent, SLOT(computeEMC()));
}
