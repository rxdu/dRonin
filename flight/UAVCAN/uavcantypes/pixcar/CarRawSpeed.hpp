/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/rdu/Workspace/dronin/dRonin/flight/UAVCAN/pixcar/20504.CarRawSpeed.uavcan
 */

#ifndef PIXCAR_CARRAWSPEED_HPP_INCLUDED
#define PIXCAR_CARRAWSPEED_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
uint32 time_stamp
uint16 hallsensor_count
float32 speed_estimate
******************************************************************************/

/********************* DSDL signature source definition ***********************
pixcar.CarRawSpeed
saturated uint32 time_stamp
saturated uint16 hallsensor_count
saturated float32 speed_estimate
******************************************************************************/

#undef time_stamp
#undef hallsensor_count
#undef speed_estimate

namespace pixcar
{

template <int _tmpl>
struct UAVCAN_EXPORT CarRawSpeed_
{
    typedef const CarRawSpeed_<_tmpl>& ParameterType;
    typedef CarRawSpeed_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 32, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > time_stamp;
        typedef ::uavcan::IntegerSpec< 16, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > hallsensor_count;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > speed_estimate;
    };

    enum
    {
        MinBitLen
            = FieldTypes::time_stamp::MinBitLen
            + FieldTypes::hallsensor_count::MinBitLen
            + FieldTypes::speed_estimate::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::time_stamp::MaxBitLen
            + FieldTypes::hallsensor_count::MaxBitLen
            + FieldTypes::speed_estimate::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::time_stamp >::Type time_stamp;
    typename ::uavcan::StorageType< typename FieldTypes::hallsensor_count >::Type hallsensor_count;
    typename ::uavcan::StorageType< typename FieldTypes::speed_estimate >::Type speed_estimate;

    CarRawSpeed_()
        : time_stamp()
        , hallsensor_count()
        , speed_estimate()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<80 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 20504 };

    static const char* getDataTypeFullName()
    {
        return "pixcar.CarRawSpeed";
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
bool CarRawSpeed_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        time_stamp == rhs.time_stamp &&
        hallsensor_count == rhs.hallsensor_count &&
        speed_estimate == rhs.speed_estimate;
}

template <int _tmpl>
bool CarRawSpeed_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(time_stamp, rhs.time_stamp) &&
        ::uavcan::areClose(hallsensor_count, rhs.hallsensor_count) &&
        ::uavcan::areClose(speed_estimate, rhs.speed_estimate);
}

template <int _tmpl>
int CarRawSpeed_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
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
    res = FieldTypes::hallsensor_count::encode(self.hallsensor_count, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::speed_estimate::encode(self.speed_estimate, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int CarRawSpeed_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
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
    res = FieldTypes::hallsensor_count::decode(self.hallsensor_count, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::speed_estimate::decode(self.speed_estimate, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature CarRawSpeed_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xF8E75C19FDF2E74AULL);

    FieldTypes::time_stamp::extendDataTypeSignature(signature);
    FieldTypes::hallsensor_count::extendDataTypeSignature(signature);
    FieldTypes::speed_estimate::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef CarRawSpeed_<0> CarRawSpeed;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::pixcar::CarRawSpeed > _uavcan_gdtr_registrator_CarRawSpeed;

}

} // Namespace pixcar

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::pixcar::CarRawSpeed >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::pixcar::CarRawSpeed::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::pixcar::CarRawSpeed >::stream(Stream& s, ::pixcar::CarRawSpeed::ParameterType obj, const int level)
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
    YamlStreamer< ::pixcar::CarRawSpeed::FieldTypes::time_stamp >::stream(s, obj.time_stamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "hallsensor_count: ";
    YamlStreamer< ::pixcar::CarRawSpeed::FieldTypes::hallsensor_count >::stream(s, obj.hallsensor_count, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "speed_estimate: ";
    YamlStreamer< ::pixcar::CarRawSpeed::FieldTypes::speed_estimate >::stream(s, obj.speed_estimate, level + 1);
}

}

namespace pixcar
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::pixcar::CarRawSpeed::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::pixcar::CarRawSpeed >::stream(s, obj, 0);
    return s;
}

} // Namespace pixcar

#endif // PIXCAR_CARRAWSPEED_HPP_INCLUDED