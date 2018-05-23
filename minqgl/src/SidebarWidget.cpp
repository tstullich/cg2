#include "SidebarWidget.hpp"

const QString SidebarWidget::CURRENT_VALUE_LABEL = "Current Value: ";

SidebarWidget::SidebarWidget(QWidget* parent) : QDockWidget(parent) {
  setWindowTitle("Settings");
  setAllowedAreas(Qt::RightDockWidgetArea);

  container = new QWidget();
  layout = new QVBoxLayout();

  initDrawPointsBox();
  initControlPointsBox();
  initBezierSurfaceBox();
  initMLSSurfaceBox();

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

void SidebarWidget::initDrawPointsBox() {
  drawPointsBox = new QCheckBox("Draw Points", this);
}

void SidebarWidget::initControlPointsBox() {
  drawRegularGridBox = new QCheckBox("Draw regular grid", this);
  drawControlPointsMeshBox = new QCheckBox("Draw control points mesh", this);

  gridXBox = new QSpinBox(this);
  gridXBox->setRange(1, 100);

  gridYBox = new QSpinBox(this);
  gridYBox->setRange(1, 100);

  radiusBox = new QDoubleSpinBox(this);
  radiusBox->setRange(0.0, 10.0);
}

void SidebarWidget::initBezierSurfaceBox() {
  drawBezierSurfaceBox = new QCheckBox("Draw bezier surface", this);

  bezierSubDivisionsBox = new QSpinBox(this);
  bezierSubDivisionsBox->setRange(1, 20);
}

void SidebarWidget::initMLSSurfaceBox() {
  drawMLSSurfaceBox = new QCheckBox("Draw MLS surface", this);

  mlsSubDivisionsBox = new QSpinBox(this);
  mlsSubDivisionsBox->setRange(1, 20);
}
