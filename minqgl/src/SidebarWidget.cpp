#include "SidebarWidget.hpp"

const QString SidebarWidget::CURRENT_VALUE_LABEL = "Current Value: ";

SidebarWidget::SidebarWidget(QWidget* parent) : QDockWidget(parent) {
  setWindowTitle("Settings");
  setAllowedAreas(Qt::RightDockWidgetArea);

  container = new QWidget();
  layout = new QVBoxLayout();

  initDrawPointsBox(parent);
  initControlPointsBox(parent);
  initBezierSurfaceBox(parent);
  initMLSSurfaceBox(parent);

  auto pointsBox = new QGroupBox("Points");
  auto pointsLayout = new QVBoxLayout;
  pointsLayout->addWidget(drawPointsBox);
  pointsBox->setLayout(pointsLayout);

  // Control Points Section
  auto controlPointsBox = new QGroupBox("Control Points");
  auto controlPointsLayout = new QVBoxLayout;
  controlPointsLayout->addWidget(drawRegularGridBox);
  controlPointsLayout->addWidget(drawControlPointsMeshBox);

  auto gridSizeLayout = new QHBoxLayout;
  gridSizeLayout->addWidget(new QLabel("Grid size:"));
  gridSizeLayout->addWidget(gridXBox);
  gridSizeLayout->addWidget(new QLabel("x"));
  gridSizeLayout->addWidget(gridYBox);

  auto gridPointsWrapper = new QWidget;
  gridPointsWrapper->setLayout(gridSizeLayout);

  auto radiusBoxLayout = new QHBoxLayout;
  radiusBoxLayout->addWidget(new QLabel("Radius:"));
  radiusBoxLayout->addWidget(radiusBox);

  auto radiusWrapper = new QWidget;
  radiusWrapper->setLayout(radiusBoxLayout);

  controlPointsLayout->addWidget(gridPointsWrapper);
  controlPointsLayout->addWidget(radiusWrapper);
  controlPointsBox->setLayout(controlPointsLayout);

  // Bezier Surface Section
  auto bezierSurfaceBox = new QGroupBox("Bezier Surface");
  auto bezierSurfaceLayout = new QVBoxLayout;
  bezierSurfaceLayout->addWidget(drawBezierSurfaceBox);

  auto bezierSubdivisionsLayout = new QHBoxLayout;
  bezierSubdivisionsLayout->addWidget(new QLabel("Subdivisions k"));
  bezierSubdivisionsLayout->addWidget(bezierSubDivisionsBox);

  auto bezierSubdivisionsWrapper = new QWidget;
  bezierSubdivisionsWrapper->setLayout(bezierSubdivisionsLayout);
  bezierSurfaceLayout->addWidget(bezierSubdivisionsWrapper);
  bezierSurfaceBox->setLayout(bezierSurfaceLayout);

  // MLS Surface
  auto mlsSurfaceBox = new QGroupBox("MLS Surface");
  auto mlsSurfaceLayout = new QVBoxLayout;
  mlsSurfaceLayout->addWidget(drawMLSSurfaceBox);

  auto mlsSubdivisionsLayout = new QHBoxLayout;
  mlsSubdivisionsLayout->addWidget(new QLabel("Subdivisions k"));
  mlsSubdivisionsLayout->addWidget(mlsSubDivisionsBox);

  auto mlsSubDivisionsWrapper = new QWidget;
  mlsSubDivisionsWrapper->setLayout(mlsSubdivisionsLayout);
  mlsSurfaceLayout->addWidget(mlsSubDivisionsWrapper);
  mlsSurfaceBox->setLayout(mlsSurfaceLayout);

  // Add all the widgets to the final layout
  layout->addWidget(pointsBox);
  layout->addWidget(controlPointsBox);
  layout->addWidget(bezierSurfaceBox);
  layout->addWidget(mlsSurfaceBox);
  layout->setSizeConstraint(QLayout::SetFixedSize);

  container->setLayout(layout);
  setWidget(container);
}

void SidebarWidget::initDrawPointsBox(QWidget *parent) {
  drawPointsBox = new QCheckBox("Draw Points", this);
  drawPointsBox->setCheckState(Qt::Checked);

  connect(drawPointsBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawPoints(bool)));
}

void SidebarWidget::initControlPointsBox(QWidget *parent) {
  drawRegularGridBox = new QCheckBox("Draw regular grid", this);
  drawRegularGridBox->setCheckState(Qt::Checked);
  connect(drawRegularGridBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawRegularGrid(bool)));

  drawControlPointsMeshBox = new QCheckBox("Draw control points mesh", this);
  connect(drawControlPointsMeshBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawControlMeshPoints(bool)));

  gridXBox = new QSpinBox(this);
  gridXBox->setRange(1, 1000);
  gridXBox->setValue(10);
  connect(gridXBox, SIGNAL(valueChanged(int)), parent, SLOT(setGridXDim(int)));

  gridYBox = new QSpinBox(this);
  gridYBox->setRange(1, 1000);
  gridYBox->setValue(10);
  connect(gridYBox, SIGNAL(valueChanged(int)), parent, SLOT(setGridYDim(int)));

  radiusBox = new QDoubleSpinBox(this);
  radiusBox->setRange(0.0, 1000.0);
  radiusBox->setValue(0.1);
  connect(radiusBox, SIGNAL(valueChanged(double)), parent, SLOT(setRadius(double)));
}

void SidebarWidget::initBezierSurfaceBox(QWidget *parent) {
  drawBezierSurfaceBox = new QCheckBox("Draw bezier surface", this);
  connect(drawBezierSurfaceBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawBezier(bool)));

  bezierSubDivisionsBox = new QSpinBox(this);
  bezierSubDivisionsBox->setRange(1, 20);
  connect(bezierSubDivisionsBox, SIGNAL(valueChanged(int)), parent, SLOT(setBezierSubdivisions(int)));
}

void SidebarWidget::initMLSSurfaceBox(QWidget *parent) {
  drawMLSSurfaceBox = new QCheckBox("Draw MLS surface", this);
  connect(drawMLSSurfaceBox, SIGNAL(toggled(bool)), parent, SLOT(setDrawMls(bool)));

  mlsSubDivisionsBox = new QSpinBox(this);
  mlsSubDivisionsBox->setRange(1, 20);
  connect(mlsSubDivisionsBox, SIGNAL(valueChanged(int)), parent, SLOT(setMlsSubdivisions(int)));
}
