/****************************************************************************
** Meta object code from reading C++ file 'PubMsg.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/PubMsg.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PubMsg.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PubMsg[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x05,
      30,   22,    7,    7, 0x05,

 // slots: signature, parameters, type, tag, flags
      53,   22,    7,    7, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_PubMsg[] = {
    "PubMsg\0\0rosShutdown()\0lat,lon\0"
    "ReplanMsg(float,float)\0Replanning(float,float)\0"
};

void PubMsg::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PubMsg *_t = static_cast<PubMsg *>(_o);
        switch (_id) {
        case 0: _t->rosShutdown(); break;
        case 1: _t->ReplanMsg((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 2: _t->Replanning((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PubMsg::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PubMsg::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_PubMsg,
      qt_meta_data_PubMsg, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PubMsg::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PubMsg::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PubMsg::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PubMsg))
        return static_cast<void*>(const_cast< PubMsg*>(this));
    return QThread::qt_metacast(_clname);
}

int PubMsg::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void PubMsg::rosShutdown()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void PubMsg::ReplanMsg(float _t1, float _t2)const
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(const_cast< PubMsg *>(this), &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
