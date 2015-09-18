/****************************************************************************
** Meta object code from reading C++ file 'PickingViewer.h'
**
** Created: Mon Jun 10 15:04:30 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "PickingViewer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PickingViewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PickingViewer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x08,
      37,   14,   14,   14, 0x08,
      59,   14,   14,   14, 0x08,
      83,   14,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PickingViewer[] = {
    "PickingViewer\0\0turnOnSelectionFace()\0"
    "turnOnSelectionEdge()\0turnOnSelectionVertex()\0"
    "turnOffSelection()\0"
};

void PickingViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PickingViewer *_t = static_cast<PickingViewer *>(_o);
        switch (_id) {
        case 0: _t->turnOnSelectionFace(); break;
        case 1: _t->turnOnSelectionEdge(); break;
        case 2: _t->turnOnSelectionVertex(); break;
        case 3: _t->turnOffSelection(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData PickingViewer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PickingViewer::staticMetaObject = {
    { &QtViewer::staticMetaObject, qt_meta_stringdata_PickingViewer,
      qt_meta_data_PickingViewer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PickingViewer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PickingViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PickingViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PickingViewer))
        return static_cast<void*>(const_cast< PickingViewer*>(this));
    return QtViewer::qt_metacast(_clname);
}

int PickingViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QtViewer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
