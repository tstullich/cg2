#ifndef SIDEBARWIDGET_HPP
#define SIDEBARWIDGET_HPP

#include <iostream>

#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QLabel>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>

class SidebarWidget : public QDockWidget {
  Q_OBJECT
public:
  SidebarWidget(QWidget *parent = 0);

  /*
   * Retrieves the current mode of displaying
   * the K-d tree.
   * The options are as follows:
   * 0 - Hyper plane mode
   * 1 - Collect in radius mode
   * 2 - K-Nearest neighbor mode
   */
  int getCurrentMode() const;

  /*
   * Method to retriew the value of the slider
   */
  int getSliderValue() const;

private slots:
  /*
   * A slot that will help update our slider ranges
   * The option value should map to the following:
   * 0 - Hyper plane mode
   * 1 - Collect in radius mode
   * 2 - K-Nearest neighbor mode
   */
  void updateSliderValues(int option);

  void setCurrentValueLabel(int value);

  /*
   * Special case where we want to set the
   * value of K-Nearest Max to the size of the
   * point list
   */
  void setKNearestMax(int value);

private:
  void initModeDropdown(QWidget* parent);
  QWidget* initModeSlider();
  void initCurrentValueLabel();
  void initLinearSearchBox(QWidget* parent);

  void setSliderRange(int minVal, int maxVal);

  const int HYPER_PLANE_DEFAULT_MIN = 0;
  const int HYPER_PLANE_DEFAULT_MAX = 8;

  const int COLLECT_IN_RADIUS_MIN = 0;
  const int COLLECT_IN_RADIUS_MAX = 100;

  const int K_NEAREST_MIN = 0;
  int kNearestMax = 0;
  int sliderOption = 0;

  static const QString CURRENT_VALUE_LABEL;

  QCheckBox *linearSearchBox;
  QComboBox *dropdownMenu;
  QLabel *startLabel;
  QLabel *endLabel;
  QLabel *currentValue;
  QSlider *slider;
  QVBoxLayout *layout;
  QWidget *container;
};
#endif // SIDEBARWIDGET_HPP
