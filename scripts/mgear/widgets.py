"""mGear Qt custom widgets"""

from mgear.vendor.Qt import QtCore, QtWidgets


#################################################
# CUSTOM WIDGETS
#################################################

class TableWidgetDragRows(QtWidgets.QTableWidget):
    """qTableWidget with drag and drop functionality"""

    def __init__(self, *args, **kwargs):
        super(TableWidgetDragRows, self).__init__(*args, **kwargs)

        self.setDragEnabled(True)
        self.setAcceptDrops(True)
        self.viewport().setAcceptDrops(True)
        self.setDragDropOverwriteMode(False)
        self.setDropIndicatorShown(True)

        self.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)

    def dropEvent(self, event):
        if not event.isAccepted() and event.source() == self:
            drop_row = self.drop_on(event)

            rows = sorted(set(item.row() for item in self.selectedItems()))
            rows_to_move = [[QtWidgets.QTableWidgetItem(
                self.item(row_index, column_index))
                for column_index in range(self.columnCount())]
                for row_index in rows]

            for row_index in reversed(rows):
                self.removeRow(row_index)
                if row_index < drop_row:
                    drop_row -= 1

            for row_index, data in enumerate(rows_to_move):
                row_index += drop_row
                self.insertRow(row_index)
                for column_index, column_data in enumerate(data):
                    self.setItem(row_index, column_index, column_data)
            event.accept()
            for row_index in range(len(rows_to_move)):
                for column_index in range(self.columnCount()):
                    self.item(drop_row + row_index,
                              column_index).setSelected(True)

    def drop_on(self, event):
        index = self.indexAt(event.pos())
        if not index.isValid():
            return self.rowCount()

        return index.row() + 1 if self.is_below(event.pos(),
                                                index) else index.row()

    def is_below(self, pos, index):
        rect = self.visualRect(index)
        margin = 2
        if pos.y() - rect.top() < margin:
            return False
        elif rect.bottom() - pos.y() < margin:
            return True
        return rect.contains(pos, True) \
            and not (int(self.model().flags(index))
                     & QtCore.Qt.ItemIsDropEnabled) \
            and pos.y() >= rect.center().y()

    def getSelectedRowsFast(self):
        selRows = []
        for item in self.selectedItems():
            if item.row() not in selRows:
                selRows.append(item.row())
        return selRows

    def droppingOnItself(self, event, index):
        dropAction = event.dropAction()

        if self.dragDropMode() == QtWidgets.QAbstractItemView.InternalMove:
            dropAction = QtCore.Qt.MoveAction

        if (event.source() == self
            and event.possibleActions() & QtCore.Qt.MoveAction
                and dropAction == QtCore.Qt.MoveAction):
            selectedIndexes = self.selectedIndexes()
            child = index
            while child.isValid() and child != self.rootIndex():
                if child in selectedIndexes:
                    return True
                child = child.parent()

        return False

    def dropOn(self, event):
        if event.isAccepted():
            return False, None, None, None

        index = QtWidgets.QModelIndex()
        row = -1
        col = -1

        if self.viewport().rect().contains(event.pos()):
            index = self.indexAt(event.pos())
            if (not index.isValid()
                    or not self.visualRect(index).contains(event.pos())):
                index = self.rootIndex()

        if self.model().supportedDropActions() & event.dropAction():
            if index != self.rootIndex():
                dropIndicatorPosition = self.position(event.pos(),
                                                      self.visualRect(index),
                                                      index)
                qabw = QtWidgets.QAbstractItemView
                if dropIndicatorPosition == qabw.AboveItem:
                    row = index.row()
                    col = index.column()
                elif dropIndicatorPosition == qabw.BelowItem:
                    row = index.row() + 1
                    col = index.column()
                else:
                    row = index.row()
                    col = index.column()

            if not self.droppingOnItself(event, index):
                return True, row, col, index

        return False, None, None, None

    def position(self, pos, rect, index):
        r = QtWidgets.QAbstractItemView.OnViewport
        margin = 5
        if pos.y() - rect.top() < margin:
            r = QtWidgets.QAbstractItemView.AboveItem
        elif rect.bottom() - pos.y() < margin:
            r = QtWidgets.QAbstractItemView.BelowItem
        elif rect.contains(pos, True):
            r = QtWidgets.QAbstractItemView.OnItem

        if (r == QtWidgets.QAbstractItemView.OnItem
                and not (self.model().flags(index)
                         & QtCore.Qt.ItemIsDropEnabled)):
            if pos.y() < rect.center().y():
                r = QtWidgets.QAbstractItemView.AboveItem
            else:
                r = QtWidgets.QAbstractItemView.BelowItem

        return r
