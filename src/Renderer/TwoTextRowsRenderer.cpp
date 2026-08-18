// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TwoTextRowsRenderer.hpp"
#include "ui/canvas/Canvas.hpp"
#include "ui/canvas/TextFormat.hpp"
#include "Screen/Layout.hpp"
#include "ui/dim/Rect.hpp"

#include <algorithm>

unsigned
TwoTextRowsRenderer::CalculateLayout(const Font &_first_font,
                                     const Font &_second_font) noexcept
{
  const unsigned first_font_height = _first_font.GetHeight();
  const unsigned second_font_height = _second_font.GetHeight();
  const unsigned text_padding = Layout::GetTextPadding();
  const unsigned max_height = Layout::GetMaximumControlHeight();
  const unsigned padded_height =
    first_font_height + second_font_height + 3 * text_padding;
  const unsigned row_height = std::max(padded_height, max_height);

  unsigned vertical_padding =
    (row_height - first_font_height - second_font_height) / 3;

  first_font = &_first_font;
  second_font = &_second_font;
  x = text_padding;
  first_y = vertical_padding;
  second_y = first_y + first_font_height + vertical_padding;

  return row_height;
}

void
TwoTextRowsRenderer::PrepareRow(const PixelRect &rc) const noexcept
{
  if (rc.top == edge_row_top)
    return;

  edge_row_top = rc.top;
  first_row_right_edge = second_row_right_edge = 0;
}

void
TwoTextRowsRenderer::DrawFirstRow(Canvas &canvas, const PixelRect &rc,
                                  const char *text) const noexcept
{
  PrepareRow(rc);

  canvas.Select(*first_font);
  first_row_right_edge = rc.left + x + (int)canvas.CalcTextWidth(text);
  canvas.DrawClippedText({rc.left + x, rc.top + first_y}, rc, text);
}

void
TwoTextRowsRenderer::DrawSecondRow(Canvas &canvas, const PixelRect &rc,
                                   const char *text) const noexcept
{
  PrepareRow(rc);

  canvas.Select(*second_font);
  second_row_right_edge = rc.left + x + (int)canvas.CalcTextWidth(text);

  // leave the same margin on the right as the left inset, so wrapped
  // text doesn't run to the very edge of the row
  const int right = std::max(rc.left + x, rc.right - x);
  const PixelRect wrap_rc{rc.left + x, rc.top, right, rc.top};
  const int max_height = 2 * (int)second_font->GetLineSpacing();
  const int height = std::min(max_height,
                              (int)canvas.DrawFormattedText(wrap_rc, text,
                                                            DT_CALCRECT));

  // if wrapping the text pushes it past the bottom of the row, move
  // it up (but not into the first row) so it still fits; never grow
  // beyond two lines (drawn with this capped height, so a third line
  // is clipped rather than attempted)
  int top = second_y;
  const int available = rc.bottom - rc.top - top;
  if (height > available) {
    const int min_top = first_y + (int)first_font->GetHeight();
    top = std::max(min_top, rc.bottom - rc.top - height);
  }

  const PixelRect text_rc{rc.left + x, rc.top + top,
                          right, rc.top + top + height};
  canvas.DrawFormattedText(text_rc, text, DT_LEFT);
}

int
TwoTextRowsRenderer::DrawRightFirstRow(Canvas &canvas, const PixelRect &rc,
                                       const char *text) const noexcept
{
  PrepareRow(rc);

  canvas.Select(*second_font);
  int text_width = canvas.CalcTextWidth(text);
  int text_x = rc.right - x - text_width;
  if (text_x < rc.left)
    /* text is too large: skip it completely (is there something
       better we can do?) */
    return rc.right;

  /* skip if it would overlap the left text from DrawFirstRow() */
  if (first_row_right_edge > 0 && text_x < first_row_right_edge + x)
    return rc.right;

  canvas.DrawText({text_x, rc.top + first_y}, text);
  return text_x - x;
}

int
TwoTextRowsRenderer::DrawRightSecondRow(Canvas &canvas, const PixelRect &rc,
                                        const char *text) const noexcept
{
  PrepareRow(rc);

  canvas.Select(*second_font);
  int text_width = canvas.CalcTextWidth(text);
  int text_x = rc.right - x - text_width;
  if (text_x < rc.left)
    /* text is too large: skip it completely (is there something
       better we can do?) */
    return rc.right;

  /* skip if it would overlap the left text from DrawSecondRow() */
  if (second_row_right_edge > 0 && text_x < second_row_right_edge + x)
    return rc.right;

  canvas.DrawText({text_x, rc.top + second_y}, text);
  return text_x - x;
}
