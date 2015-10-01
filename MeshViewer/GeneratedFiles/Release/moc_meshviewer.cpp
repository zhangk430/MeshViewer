/****************************************************************************
** Meta object code from reading C++ file 'meshviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../meshviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'meshviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MeshViewer_t {
    QByteArrayData data[10];
    char stringdata0[85];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MeshViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MeshViewer_t qt_meta_stringdata_MeshViewer = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MeshViewer"
QT_MOC_LITERAL(1, 11, 4), // "load"
QT_MOC_LITERAL(2, 16, 0), // ""
QT_MOC_LITERAL(3, 17, 6), // "import"
QT_MOC_LITERAL(4, 24, 4), // "save"
QT_MOC_LITERAL(5, 29, 13), // "showWireFrame"
QT_MOC_LITERAL(6, 43, 4), // "show"
QT_MOC_LITERAL(7, 48, 15), // "showMeshChanged"
QT_MOC_LITERAL(8, 64, 3), // "row"
QT_MOC_LITERAL(9, 68, 16) // "setChosenElement"

    },
    "MeshViewer\0load\0\0import\0save\0showWireFrame\0"
    "show\0showMeshChanged\0row\0setChosenElement"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MeshViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x08 /* Private */,
       3,    0,   50,    2, 0x08 /* Private */,
       4,    0,   51,    2, 0x08 /* Private */,
       5,    1,   52,    2, 0x08 /* Private */,
       7,    1,   55,    2, 0x08 /* Private */,
       7,    0,   58,    2, 0x08 /* Private */,
       9,    0,   59,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MeshViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MeshViewer *_t = static_cast<MeshViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->load(); break;
        case 1: _t->import(); break;
        case 2: _t->save(); break;
        case 3: _t->showWireFrame((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->showMeshChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->showMeshChanged(); break;
        case 6: _t->setChosenElement(); break;
        default: ;
        }
    }
}

const QMetaObject MeshViewer::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MeshViewer.data,
      qt_meta_data_MeshViewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MeshViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MeshViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MeshViewer.stringdata0))
        return static_cast<void*>(const_cast< MeshViewer*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MeshViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
