#ifndef SIDEBARWIDGET_HPP
#define SIDEBARWIDGET_HPP

#include <iostream>

#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>

class SidebarWidget : public QDockWidget {
  Q_OBJECT
public:
  SidebarWidget(QWidget *parent = 0);

private:
  void initMeshBox(QWidget *parent);
  void initNormalCalculationBox(QWidget *parent);
  void initGraphLaplaceBox(QWidget *parent);
  void initCotanLaplaceBox(QWidget *parent);

  const int HYPER_PLANE_DEFAULT_MIN = 0;
  const int HYPER_PLANE_DEFAULT_MAX = 8;

  const int COLLECT_IN_RADIUS_MIN = 0;
  const int COLLECT_IN_RADIUS_MAX = 100;

  const int K_NEAREST_MIN = 0;
  int kNearestMax = 0;
  int sliderOption = 0;

  static const QString CURRENT_VALUE_LABEL;

  QCheckBox *drawMeshBox;
  QDoubleSpinBox *meshAlphaBox;

  QCheckBox *drawUnweightedNormalsBox;
  QCheckBox *drawWeightedNormalsBox;

  QCheckBox *drawGraphLaplaceBox;
  QDoubleSpinBox *stepSizeBox;
  QPushButton *graphLaplaceMoveButton;
  QPushButton *graphLaplaceResetButton;

  QCheckBox *drawCotanLaplaceBox;
  QDoubleSpinBox *explicitStepBox;
  QPushButton *explicitStepButton;
  QDoubleSpinBox *implicitStepBox;
  QPushButton *implicitStepButton;
  QSpinBox *basisFunctionsBox;
  QCheckBox *manifoldHarmonicsBox;
  QPushButton *cotanLaplaceResetButton;

  QVBoxLayout *layout;
  QWidget *container;
};
#endif // SIDEBARWIDGET_HPP
