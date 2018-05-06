#include "SidebarWidget.hpp"

SidebarWidget::SidebarWidget(QWidget *parent) : QDockWidget(parent) {
  // Create our siede dock. Would be good to put this into own widget later
  // auto sideDock = new QDockWidget("K-d Tree Settings", parent);
  setWindowTitle("K-d Tree Settings");
  setAllowedAreas(Qt::RightDockWidgetArea);

  container = new QWidget();
  layout = new QVBoxLayout();

  QStringList dropdownOptions = {"Hyper Planes", "Radius", "K-Nearest"};
  dropdownMenu = new QComboBox(parent);
  dropdownMenu->addItems(dropdownOptions);

  connect(dropdownMenu, SIGNAL(currentIndexChanged(int)), this,
          SLOT(updateSliderValues(int)));
  connect(dropdownMenu, SIGNAL(currentIndexChanged(int)), parent,
          SLOT(setDrawMode(int)));

  slider = new QSlider(Qt::Horizontal);
  slider->setTickPosition(QSlider::TicksBothSides);
  slider->setTickInterval(1);
  slider->setSingleStep(1);
  // Setting the min and max for the hyper plane drawing mode
  slider->setRange(HYPER_PLANE_DEFAULT_MIN, HYPER_PLANE_DEFAULT_MAX);

  // Wrap slider so we can use labels
  startLabel = new QLabel(QString::number(HYPER_PLANE_DEFAULT_MIN), this);
  endLabel = new QLabel(QString::number(HYPER_PLANE_DEFAULT_MAX), this);
  auto sliderLayout = new QGridLayout();
  sliderLayout->addWidget(slider, 0, 0, 1, 4);
  sliderLayout->addWidget(startLabel, 1, 0, 1, 1);
  sliderLayout->addWidget(endLabel, 1, 3, 1, 1);

  auto sliderContainer = new QWidget();
  sliderContainer->setLayout(sliderLayout);

  layout->addWidget(dropdownMenu);
  layout->addWidget(sliderContainer);
  layout->setSizeConstraint(QLayout::SetFixedSize);

  container->setLayout(layout);
  setWidget(container);
}

int SidebarWidget::getCurrentMode() const {
  return dropdownMenu->currentIndex();
}

void SidebarWidget::setSliderCallback(QWidget *widget) {
  connect(slider, SIGNAL(valueChanged(int)), widget,
          SLOT(sliderValueChanged(int)));
}

void SidebarWidget::setSliderRange(int minVal, int maxVal) {
  slider->setRange(minVal, maxVal);
  startLabel->setText(QString::number(minVal));
  endLabel->setText(QString::number(maxVal));
}

void SidebarWidget::updateSliderValues(int option) {
  if (option == 0) {
    // This is our hyper plane drawing mode
    setSliderRange(HYPER_PLANE_DEFAULT_MIN, HYPER_PLANE_DEFAULT_MAX);
  } else if (option == 1) {
    // This is our collect in radius drawing mode
    setSliderRange(COLLECT_IN_RADIUS_MIN, COLLECT_IN_RADIUS_MAX);
  } else {
    // This is our collect nearest neighbor drawing mode
    setSliderRange(K_NEAREST_MIN, K_NEAREST_MAX);
  }
}
