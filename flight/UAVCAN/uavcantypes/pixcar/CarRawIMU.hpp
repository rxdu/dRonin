/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/dronin/dRonin/flight/UAVCAN/pixcar/25002.CarRawIMU.uavcan
 */

#ifndef PIXCAR_CARRAWIMU_HPP_INCLUDED
#define PIXCAR_CARRAWIMU_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
# 4-byte float
uint32 time_stamp
float32[3] gyro
float32[3] accel
******************************************************************************/

/********************* DSDL signature source definition ***********************
pixcar.CarRawIMU
saturated uint32 time_stamp
saturated float32[3] gyro
saturated float32[3] accel
******************************************************************************/

#undef time_stamp
#undef gyro
#undef accel

namespace pixcar
{

template <int _tmpl>
struct UAVCAN_EXPORT CarRawIMU_
{
    typedef const CarRawIMU_<_tmpl>& ParameterType;
    typedef CarRawIMU_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > time_stamp;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > gyro;
        typedef ::uavcan::Array< ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeStatic, 3 > accel;
    };

    enum
    {
        MinBitLen
            = FieldTypes::time_stamp::MinBitLen
            + FieldTypes::gyro::MinBitLen
            + FieldTypes::accel::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::time_stamp::MaxBitLen
            + FieldTypes::gyro::MaxBitLen
            + FieldTypes::accel::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::time_stamp >::Type time_stamp;
    typename ::uavcan::StorageType< typename FieldTypes::gyro >::Type gyro;
    typename ::uavcan::StorageType< typename FieldTypes::accel >::Type accel;

    CarRawIMU_()
        : time_stamp()
        , gyro()
        , accel()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<224 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 25002 };

    static const char* getDataTypeFullName()
    {
        return "pixcar.CarRawIMU";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool CarRawIMU_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        time_stamp == rhs.time_stamp &&
        gyro == rhs.gyro &&
        accel == rhs.accel;
}

template <int _tmpl>
bool CarRawIMU_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(time_stamp, rhs.time_stamp) &&
        ::uavcan::areClose(gyro, rhs.gyro) &&
        ::uavcan::areClose(accel, rhs.accel);
}

template <int _tmpl>
int CarRawIMU_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::time_stamp::encode(self.time_stamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro::encode(self.gyro, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::accel::encode(self.accel, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int CarRawIMU_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::time_stamp::decode(self.time_stamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::gyro::decode(self.gyro, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::accel::decode(self.accel, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature CarRawIMU_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x938CF09578C5A762ULL);

    FieldTypes::time_stamp::extendDataTypeSignature(signature);
    FieldTypes::gyro::extendDataTypeSignature(signature);
    FieldTypes::accel::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef CarRawIMU_<0> CarRawIMU;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::pixcar::CarRawIMU > _uavcan_gdtr_registrator_CarRawIMU;

}

} // Namespace pixcar

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::pixcar::CarRawIMU >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::pixcar::CarRawIMU::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::pixcar::CarRawIMU >::stream(Stream& s, ::pixcar::CarRawIMU::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "time_stamp: ";
    YamlStreamer< ::pixcar::CarRawIMU::FieldTypes::time_stamp >::stream(s, obj.time_stamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "gyro: ";
    YamlStreamer< ::pixcar::CarRawIMU::FieldTypes::gyro >::stream(s, obj.gyro, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "accel: ";
    YamlStreamer< ::pixcar::CarRawIMU::FieldTypes::accel >::stream(s, obj.accel, level + 1);
}

}

namespace pixcar
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::pixcar::CarRawIMU::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::pixcar::CarRawIMU >::stream(s, obj, 0);
    return s;
}

} // Namespace pixcar

#endif // PIXCAR_CARRAWIMU_HPP_INCLUDED