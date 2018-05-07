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

  // Connecting signals to slots here so our values update correctly when a new
  // value is selected from the dropdown
  connect(dropdownMenu, SIGNAL(currentIndexChanged(int)), this,
          SLOT(updateSliderValues(int)));
  connect(dropdownMenu, SIGNAL(currentIndexChanged(int)), parent,
          SLOT(setDrawMode(int)));

  slider = new QSlider(Qt::Horizontal);
  slider->setTickPosition(QSlider::NoTicks);
  slider->setTickInterval(1);
  slider->setSingleStep(1);
  // Setting the min and max for the hyper plane drawing mode
  slider->setRange(HYPER_PLANE_DEFAULT_MIN, HYPER_PLANE_DEFAULT_MAX);

  startLabel = new QLabel(QString::number(HYPER_PLANE_DEFAULT_MIN), this);
  endLabel = new QLabel(QString::number(HYPER_PLANE_DEFAULT_MAX), this);
  // Wrap slider in a grid layout so we can add labels easily
  auto sliderLayout = new QGridLayout();
  sliderLayout->addWidget(slider, 0, 0, 1, 4);
  sliderLayout->addWidget(startLabel, 1, 0, 1, 1);
  sliderLayout->addWidget(endLabel, 1, 3, 1, 1);

  // This should be called
  connect(parent, SIGNAL(kNearestChanged(int)), this,
          SLOT(setKNearestMax(int)));

  auto sliderContainer = new QWidget();
  sliderContainer->setLayout(sliderLayout);

  // Configure our checkbox for performing a linear search
  linearSearchBox = new QCheckBox("Perform Linear Search", this);
  connect(linearSearchBox, SIGNAL(toggled(bool)), parent,
          SLOT(setPerformLinearSearch(bool)));

  // Add all the widgets to the final layout
  layout->addWidget(dropdownMenu);
  layout->addWidget(sliderContainer);
  layout->addWidget(linearSearchBox);
  layout->setSizeConstraint(QLayout::SetFixedSize);

  container->setLayout(layout);
  setWidget(container);
}

int SidebarWidget::getCurrentMode() const {
  return dropdownMenu->currentIndex();
}

int SidebarWidget::getSliderValue() const { return slider->value(); }

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
  sliderOption = option;
  if (sliderOption == 0) {
    // This is our hyper plane drawing mode
    setSliderRange(HYPER_PLANE_DEFAULT_MIN, HYPER_PLANE_DEFAULT_MAX);
  } else if (sliderOption == 1) {
    // This is our collect in radius drawing mode
    setSliderRange(COLLECT_IN_RADIUS_MIN, COLLECT_IN_RADIUS_MAX);
  } else {
    // This is our collect nearest neighbor drawing mode
    setSliderRange(K_NEAREST_MIN, kNearestMax);
  }
}

void SidebarWidget::setKNearestMax(int value) {
  kNearestMax = value;
  // Need to internally update this
  updateSliderValues(sliderOption);
}
