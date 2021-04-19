def debug_printing(debug_dict):

    for key in debug_dict:
        print("{}: {}".format(key, round(debug_dict[key], 2)))
    print("#" * 40)


class DebugDisplay:
    def __init__(self, ot, format_str):
        # UI
        self.canvas = ot.conn.ui.stock_canvas
        self.display_panel = self.canvas.add_panel()
        ss = self.canvas.rect_transform.size
        # TODO: modularize size/position info
        self.display_panel.rect_transform.size = (280, 50)
        display_pos = (100.0 - (ss[0] / 2.0), (ss[1] / 2.0) - 60.0)
        self.display_panel.rect_transform.position = display_pos
        self.display_format_str = format_str
        self.display_text = self.display_panel.add_text("")

    # def __del__(self):
    #     self.display_panel.remove()

    def update(self, data):
        display_str = self.display_format_str.format(*data)
        self.display_text.content = display_str

