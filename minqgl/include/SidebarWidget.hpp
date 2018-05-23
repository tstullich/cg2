#ifndef SIDEBARWIDGET_HPP
#define SIDEBARWIDGET_HPP

#include <iostream>

#include <QCheckBox>
#include <QComboBox>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QVBoxLayout>

class SidebarWidget : public QDockWidget {
  Q_OBJECT
public:
  SidebarWidget(QWidget *parent = 0);

private:
  void initDrawPointsBox(QWidget *parent);
  void initControlPointsBox(QWidget *parent);
  void initBezierSurfaceBox(QWidget *parent);
  void initMLSSurfaceBox(QWidget *parent);

  const int HYPER_PLANE_DEFAULT_MIN = 0;
  const int HYPER_PLANE_DEFAULT_MAX = 8;

  const int COLLECT_IN_RADIUS_MIN = 0;
  const int COLLECT_IN_RADIUS_MAX = 100;

  const int K_NEAREST_MIN = 0;
  int kNearestMax = 0;
  int sliderOption = 0;

  static const QString CURRENT_VALUE_LABEL;

  QCheckBox *drawPointsBox;
  QCheckBox *drawRegularGridBox;
  QCheckBox *drawControlPointsMeshBox;

  QSpinBox *gridXBox;
  QSpinBox *gridYBox;
  QDoubleSpinBox *radiusBox;

  QCheckBox *drawBezierSurfaceBox;
  QSpinBox *bezierSubDivisionsBox;

  QCheckBox *drawMLSSurfaceBox;
  QSpinBox *mlsSubDivisionsBox;

  QVBoxLayout *layout;
  QWidget *container;
};
#endif // SIDEBARWIDGET_HPP
