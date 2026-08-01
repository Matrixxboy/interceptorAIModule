"""One axis speed control: label, − / slider / +, and live numeric value."""

from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QWidget,
)


class AxisSpeedRow(QWidget):
    """Compact real-time speed tuner for a single flight axis."""

    valueChanged = pyqtSignal(float)

    def __init__(
        self,
        label: str,
        tooltip: str,
        value: float,
        lo: float = 0.0,
        hi: float = 1.0,
        step: float = 0.05,
        decimals: int = 2,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._lo = float(lo)
        self._hi = float(hi)
        self._step = float(step)
        self._decimals = int(decimals)
        self._scale = 10 ** self._decimals  # slider uses integers
        self._updating = False

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 2, 0, 2)
        root.setSpacing(6)

        self.lbl = QLabel(label)
        self.lbl.setObjectName("formLabel")
        self.lbl.setFixedWidth(72)
        self.lbl.setToolTip(tooltip)
        self.setToolTip(tooltip)

        self.btn_minus = QPushButton("−")
        self.btn_minus.setObjectName("btnStep")
        self.btn_minus.setFixedSize(28, 28)
        self.btn_minus.setToolTip(f"Decrease by {step:g}")
        self.btn_minus.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(int(round(self._lo * self._scale)), int(round(self._hi * self._scale)))
        self.slider.setSingleStep(max(1, int(round(self._step * self._scale))))
        self.slider.setPageStep(max(1, int(round(self._step * self._scale * 4))))
        self.slider.setMinimumWidth(90)
        self.slider.setToolTip(tooltip)

        self.btn_plus = QPushButton("+")
        self.btn_plus.setObjectName("btnStep")
        self.btn_plus.setFixedSize(28, 28)
        self.btn_plus.setToolTip(f"Increase by {step:g}")
        self.btn_plus.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(self._lo, self._hi)
        self.spin.setSingleStep(self._step)
        self.spin.setDecimals(self._decimals)
        self.spin.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.spin.setFixedWidth(72)
        self.spin.setButtonSymbols(QDoubleSpinBox.ButtonSymbols.NoButtons)
        self.spin.setToolTip(tooltip)

        root.addWidget(self.lbl)
        root.addWidget(self.btn_minus)
        root.addWidget(self.slider, stretch=1)
        root.addWidget(self.btn_plus)
        root.addWidget(self.spin)

        self.btn_minus.clicked.connect(self._nudge_minus)
        self.btn_plus.clicked.connect(self._nudge_plus)
        self.slider.valueChanged.connect(self._on_slider)
        self.spin.valueChanged.connect(self._on_spin)

        self.set_value(value, emit=False)

    def value(self) -> float:
        return float(self.spin.value())

    def set_value(self, value: float, emit: bool = True) -> None:
        clipped = max(self._lo, min(self._hi, float(value)))
        self._updating = True
        self.spin.setValue(clipped)
        self.slider.setValue(int(round(clipped * self._scale)))
        self._updating = False
        if emit:
            self.valueChanged.emit(clipped)

    def _emit(self) -> None:
        self.valueChanged.emit(self.value())

    def _nudge_minus(self) -> None:
        self.set_value(self.value() - self._step, emit=True)

    def _nudge_plus(self) -> None:
        self.set_value(self.value() + self._step, emit=True)

    def _on_slider(self, raw: int) -> None:
        if self._updating:
            return
        self._updating = True
        self.spin.setValue(raw / self._scale)
        self._updating = False
        self._emit()

    def _on_spin(self, val: float) -> None:
        if self._updating:
            return
        self._updating = True
        self.slider.setValue(int(round(float(val) * self._scale)))
        self._updating = False
        self._emit()
