// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "WindowWidget.hpp"

#include <functional>

/**
 * A compact map-bottom control strip: one or more rows of
 * &lt; prev | centre label | next &gt; steppers.
 *
 * The centre label is tappable when a #LabelClickCallback is set.
 * Layout matches touch-sized map weather cursor bars (XCTherm, etc.).
 */
class CursorBarWidget : public WindowWidget {
public:
  using StepCallback = std::function<void(unsigned row, int delta)>;
  using LabelClickCallback = std::function<void(unsigned row)>;

  static constexpr unsigned MAX_ROWS = 4;

  /**
   * @param row_count Number of stepper rows (1..#MAX_ROWS).
   * @param height_scale Multiplier applied to #DefaultHeight, e.g. to
   * shrink an overlay's bar on a specific platform.
   */
  explicit CursorBarWidget(unsigned row_count=2,
                          float height_scale=1.0f) noexcept;

  void SetStepCallback(StepCallback cb) noexcept {
    step_callback = std::move(cb);
  }

  void SetLabelClickCallback(LabelClickCallback cb) noexcept {
    label_click_callback = std::move(cb);
  }

  /**
   * Update the centred label on @p row.
   */
  void SetRowText(unsigned row, const char *text,
                  bool available=true) noexcept;

  /**
   * Enable or disable the prev/next steppers on @p row.
   */
  void SetRowEnabled(unsigned row, bool enabled) noexcept;

  /**
   * Preferred height for @p row_count rows (includes inter-row separators).
   */
  [[gnu::const]]
  static unsigned DefaultHeight(unsigned row_count=2) noexcept;

  /**
   * #DefaultHeight for this instance's row count, scaled by #height_scale.
   */
  [[gnu::pure]]
  unsigned GetPreferredHeight() const noexcept;

  /* virtual methods from class Widget */
  PixelSize GetMinimumSize() const noexcept override;
  PixelSize GetMaximumSize() const noexcept override;
  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override;
  void Unprepare() noexcept override;
  void Show(const PixelRect &rc) noexcept override;
  void Move(const PixelRect &rc) noexcept override;

protected:
  void InvokeStep(unsigned row, int delta) const noexcept;
  void InvokeLabelClick(unsigned row) const noexcept;
  void RelayoutBar() noexcept;

private:
  class BarWindow;

  const unsigned row_count;
  const float height_scale;
  StepCallback step_callback;
  LabelClickCallback label_click_callback;
};
