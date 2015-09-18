/****************************************************************************
** Meta object code from reading C++ file 'KinectViewer.h'
**
** Created: Fri Jun 28 19:46:09 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "KinectViewer.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'KinectViewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_KinectViewer[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_KinectViewer[] = {
    "KinectViewer\0\0grab_data()\0"
};

void KinectViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        KinectViewer *_t = static_cast<KinectViewer *>(_o);
        switch (_id) {
        case 0: _t->grab_data(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData KinectViewer::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject KinectViewer::staticMetaObject = {
    { &PickingViewer::staticMetaObject, qt_meta_stringdata_KinectViewer,
      qt_meta_data_KinectViewer, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &KinectViewer::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *KinectViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *KinectViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_KinectViewer))
        return static_cast<void*>(const_cast< KinectViewer*>(this));
    return PickingViewer::qt_metacast(_clname);
}

int KinectViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PickingViewer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
