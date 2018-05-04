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

  slider = new QSlider(Qt::Horizontal);
  slider->setTickPosition(QSlider::TicksBothSides);
  slider->setTickInterval(1);
  slider->setSingleStep(1);
  // We can adjust these later
  slider->setMinimum(1);
  slider->setMaximum(30);

  layout->addWidget(dropdownMenu);
  layout->addWidget(slider);

  container->setLayout(layout);
  setWidget(container);
}
