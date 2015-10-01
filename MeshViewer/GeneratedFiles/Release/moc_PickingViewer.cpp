/****************************************************************************
** Meta object code from reading C++ file 'PickingViewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../QtViewer/PickingViewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PickingViewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_PickingViewer_t {
    QByteArrayData data[7];
    char stringdata0[101];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PickingViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PickingViewer_t qt_meta_stringdata_PickingViewer = {
    {
QT_MOC_LITERAL(0, 0, 13), // "PickingViewer"
QT_MOC_LITERAL(1, 14, 6), // "picked"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 19), // "turnOnSelectionFace"
QT_MOC_LITERAL(4, 42, 19), // "turnOnSelectionEdge"
QT_MOC_LITERAL(5, 62, 21), // "turnOnSelectionVertex"
QT_MOC_LITERAL(6, 84, 16) // "turnOffSelection"

    },
    "PickingViewer\0picked\0\0turnOnSelectionFace\0"
    "turnOnSelectionEdge\0turnOnSelectionVertex\0"
    "turnOffSelection"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PickingViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   40,    2, 0x08 /* Private */,
       4,    0,   41,    2, 0x08 /* Private */,
       5,    0,   42,    2, 0x08 /* Private */,
       6,    0,   43,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void PickingViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PickingViewer *_t = static_cast<PickingViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->picked(); break;
        case 1: _t->turnOnSelectionFace(); break;
        case 2: _t->turnOnSelectionEdge(); break;
        case 3: _t->turnOnSelectionVertex(); break;
        case 4: _t->turnOffSelection(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (PickingViewer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&PickingViewer::picked)) {
                *result = 0;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject PickingViewer::staticMetaObject = {
    { &QtViewer::staticMetaObject, qt_meta_stringdata_PickingViewer.data,
      qt_meta_data_PickingViewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *PickingViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PickingViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_PickingViewer.stringdata0))
        return static_cast<void*>(const_cast< PickingViewer*>(this));
    return QtViewer::qt_metacast(_clname);
}

int PickingViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QtViewer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void PickingViewer::picked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
