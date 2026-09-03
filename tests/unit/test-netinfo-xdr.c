/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "qemu/osdep.h"

#include "hw/netinfo/netinfo-protocol.h"
#include "hw/netinfo/netinfo-xdr.h"

static void test_getregister_network_golden(void)
{
    static const uint8_t expected[] = {
        0, 0, 0, 7, 'n', 'e', 't', 'w', 'o', 'r', 'k', 0,
    };
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    NiName name;
    uint8_t encoded[sizeof(expected)] = { 0 };

    ni_name_init(&name);
    onc_rpc_xdr_reader_init(&reader, expected, sizeof(expected));
    g_assert_true(ni_xdr_decode_name(&reader, &name));
    g_assert_cmpstr(name, ==, "network");
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    ni_name_clear(&name);

    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(encoded));
    g_assert_true(ni_xdr_encode_name(&writer, "network"));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, sizeof(expected));
    g_assert_cmpmem(encoded, sizeof(encoded), expected, sizeof(expected));
}

static void test_registration_ports_golden(void)
{
    static const uint8_t expected[] = {
        0, 0, 0, 7, 'n', 'e', 't', 'w', 'o', 'r', 'k', 0,
        0, 0, 2, 0x94, 0, 0, 2, 0x96,
    };
    NiBindRegistration registration = {
        .tag = (char *)"network",
        .addrs = { .udp_port = 660, .tcp_port = 662 },
    };
    NiBindRegistration decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    uint8_t encoded[sizeof(expected)] = { 0 };

    ni_bind_registration_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, expected, sizeof(expected));
    g_assert_true(ni_xdr_decode_bind_registration(&reader, &decoded));
    g_assert_cmpstr(decoded.tag, ==, "network");
    g_assert_cmpuint(decoded.addrs.udp_port, ==, 660);
    g_assert_cmpuint(decoded.addrs.tcp_port, ==, 662);
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));

    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(encoded));
    g_assert_true(ni_xdr_encode_bind_registration(&writer, &registration));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, sizeof(expected));
    g_assert_cmpmem(encoded, sizeof(encoded), expected, sizeof(expected));
    ni_bind_registration_clear(&decoded);
}

static void test_bind_arguments_golden(void)
{
    static const uint8_t expected[] = {
        0x0a, 0, 0, 2,
        0, 0, 0, 7, 'n', 'e', 't', 'w', 'o', 'r', 'k', 0,
        0, 0, 0, 7, 'n', 'e', 't', 'w', 'o', 'r', 'k', 0,
    };
    NiBindArgs args;
    NiBindArgs encoded_args = {
        .client_addr = 0x0a000002,
        .client_tag = (char *)"network",
        .server_tag = (char *)"network",
    };
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    uint8_t encoded[sizeof(expected)] = { 0 };

    ni_bind_args_init(&args);
    onc_rpc_xdr_reader_init(&reader, expected, sizeof(expected));
    g_assert_true(ni_xdr_decode_bind_args(&reader, &args));
    g_assert_cmpuint(args.client_addr, ==, 0x0a000002);
    g_assert_cmpstr(args.client_tag, ==, "network");
    g_assert_cmpstr(args.server_tag, ==, "network");
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    ni_bind_args_clear(&args);

    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(encoded));
    g_assert_true(ni_xdr_encode_bind_args(&writer, &encoded_args));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, sizeof(expected));
    g_assert_cmpmem(encoded, sizeof(encoded), expected, sizeof(expected));
}

static void test_root_result_golden(void)
{
    static const uint8_t expected[] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x24,
    };
    NiIdResult result;
    NiIdResult encoded_result = {
        .status = NI_OK,
        .has_id = true,
        .id = { .nii_object = 0, .nii_instance = 0x24 },
    };
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    uint8_t encoded[sizeof(expected)] = { 0 };

    ni_id_result_init(&result);
    onc_rpc_xdr_reader_init(&reader, expected, sizeof(expected));
    g_assert_true(ni_xdr_decode_id_result(&reader, &result));
    g_assert_cmpint(result.status, ==, NI_OK);
    g_assert_true(result.has_id);
    g_assert_cmpuint(result.id.nii_object, ==, 0);
    g_assert_cmpuint(result.id.nii_instance, ==, 0x24);
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    ni_id_result_clear(&result);

    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(encoded));
    g_assert_true(ni_xdr_encode_id_result(&writer, &encoded_result));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, sizeof(expected));
    g_assert_cmpmem(encoded, sizeof(encoded), expected, sizeof(expected));
}

static void test_lookup_arguments_golden(void)
{
    static const uint8_t expected[] = {
        0, 0, 0, 0, 0, 0, 0, 0x24,
        0, 0, 0, 4, 'n', 'a', 'm', 'e',
        0, 0, 0, 1, '/', 0, 0, 0,
    };
    NiLookupArgs args;
    NiLookupArgs encoded_args = {
        .id = { .nii_object = 0, .nii_instance = 0x24 },
        .key = (char *)"name",
        .value = (char *)"/",
    };
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    uint8_t encoded[sizeof(expected)] = { 0 };

    ni_lookup_args_init(&args);
    onc_rpc_xdr_reader_init(&reader, expected, sizeof(expected));
    g_assert_true(ni_xdr_decode_lookup_args(&reader, &args));
    g_assert_cmpuint(args.id.nii_object, ==, 0);
    g_assert_cmpuint(args.id.nii_instance, ==, 0x24);
    g_assert_cmpstr(args.key, ==, "name");
    g_assert_cmpstr(args.value, ==, "/");
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    ni_lookup_args_clear(&args);

    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(encoded));
    g_assert_true(ni_xdr_encode_lookup_args(&writer, &encoded_args));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, sizeof(expected));
    g_assert_cmpmem(encoded, sizeof(encoded), expected, sizeof(expected));
}

static void test_read_result_golden(void)
{
    static const uint8_t expected[] = {
        /* status NI_OK, id { 0, 0x24 }, one property */
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x24,
        0, 0, 0, 1,
        0, 0, 0, 4, 'n', 'a', 'm', 'e',
        0, 0, 0, 1, 0, 0, 0, 1, '/', 0, 0, 0,
    };
    NiPropertyListResult result;
    NiName encoded_values[] = { (char *)"/" };
    NiProperty encoded_properties[] = {
        {
            .name = (char *)"name",
            .values = { .count = 1, .values = encoded_values },
        },
    };
    NiPropertyListResult encoded_result = {
        .status = NI_OK,
        .stuff = {
            .id = { .nii_object = 0, .nii_instance = 0x24 },
            .props = {
                .count = 1,
                .properties = encoded_properties,
            },
        },
    };
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    uint8_t encoded[sizeof(expected)] = { 0 };

    ni_property_list_result_init(&result);
    onc_rpc_xdr_reader_init(&reader, expected, sizeof(expected));
    g_assert_true(ni_xdr_decode_property_list_result(&reader, &result));
    g_assert_cmpint(result.status, ==, NI_OK);
    g_assert_cmpuint(result.stuff.id.nii_instance, ==, 0x24);
    g_assert_cmpuint(result.stuff.props.count, ==, 1);
    g_assert_cmpstr(result.stuff.props.properties[0].name, ==, "name");
    g_assert_cmpuint(result.stuff.props.properties[0].values.count, ==, 1);
    g_assert_cmpstr(result.stuff.props.properties[0].values.values[0], ==, "/");
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    ni_property_list_result_clear(&result);

    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(encoded));
    g_assert_true(ni_xdr_encode_property_list_result(&writer,
                                                      &encoded_result));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, sizeof(expected));
    g_assert_cmpmem(encoded, sizeof(encoded), expected, sizeof(expected));
}

/* Keep every aggregate codec on an independently spelled wire vector. */
#define ASSERT_CODEC(name, type, init_fn, clear_fn, encode_value, expected) \
    do {                                                                    \
        type decoded;                                                       \
        OncRpcXdrReader codec_reader;                                       \
        OncRpcXdrWriter codec_writer;                                       \
        uint8_t codec_encoded[sizeof(expected)] = { 0 };                    \
        init_fn(&decoded);                                                   \
        onc_rpc_xdr_reader_init(&codec_reader, expected, sizeof(expected));  \
        g_assert_true(ni_xdr_decode_##name(&codec_reader, &decoded));        \
        g_assert_true(onc_rpc_xdr_reader_empty(&codec_reader));              \
        /* The independent golden also checks decoded field ordering. */     \
        onc_rpc_xdr_writer_init(&codec_writer, codec_encoded,               \
                                sizeof(codec_encoded));                      \
        g_assert_true(ni_xdr_encode_##name(&codec_writer, &decoded));        \
        g_assert_cmpuint(onc_rpc_xdr_writer_size(&codec_writer), ==,           \
                         sizeof(expected));                                  \
        g_assert_cmpmem(codec_encoded, sizeof(codec_encoded), expected,      \
                        sizeof(expected));                                  \
        clear_fn(&decoded);                                                   \
        memset(codec_encoded, 0, sizeof(codec_encoded));                     \
        onc_rpc_xdr_writer_init(&codec_writer, codec_encoded,               \
                                sizeof(codec_encoded));                      \
        g_assert_true(ni_xdr_encode_##name(&codec_writer, encode_value));    \
        g_assert_cmpuint(onc_rpc_xdr_writer_size(&codec_writer), ==,           \
                         sizeof(expected));                                  \
        g_assert_cmpmem(codec_encoded, sizeof(codec_encoded), expected,      \
                        sizeof(expected));                                  \
    } while (0)

#define ASSERT_NAME_CODEC(encode_value, expected)                           \
    do {                                                                    \
        NiName decoded;                                                     \
        OncRpcXdrReader codec_reader;                                       \
        OncRpcXdrWriter codec_writer;                                       \
        uint8_t codec_encoded[sizeof(expected)] = { 0 };                    \
        ni_name_init(&decoded);                                             \
        onc_rpc_xdr_reader_init(&codec_reader, expected, sizeof(expected));  \
        g_assert_true(ni_xdr_decode_name(&codec_reader, &decoded));          \
        g_assert_true(onc_rpc_xdr_reader_empty(&codec_reader));              \
        onc_rpc_xdr_writer_init(&codec_writer, codec_encoded,               \
                                sizeof(codec_encoded));                      \
        g_assert_true(ni_xdr_encode_name(&codec_writer, decoded));          \
        g_assert_cmpuint(onc_rpc_xdr_writer_size(&codec_writer), ==,           \
                         sizeof(expected));                                  \
        g_assert_cmpmem(codec_encoded, sizeof(codec_encoded), expected,      \
                        sizeof(expected));                                  \
        ni_name_clear(&decoded);                                             \
        memset(codec_encoded, 0, sizeof(codec_encoded));                     \
        onc_rpc_xdr_writer_init(&codec_writer, codec_encoded,               \
                                sizeof(codec_encoded));                      \
        g_assert_true(ni_xdr_encode_name(&codec_writer, encode_value));      \
        g_assert_cmpuint(onc_rpc_xdr_writer_size(&codec_writer), ==,           \
                         sizeof(expected));                                  \
        g_assert_cmpmem(codec_encoded, sizeof(codec_encoded), expected,      \
                        sizeof(expected));                                  \
    } while (0)

static void test_all_public_codec_shapes(void)
{
    static const uint8_t expected_status[] = { 0, 0, 0, 21 };
    static const uint8_t expected_id[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    };
    static const uint8_t expected_name[] = {
        0, 0, 0, 1, 'x', 0, 0, 0,
    };
    static const uint8_t expected_name_list[] = {
        0, 0, 0, 2, 0, 0, 0, 1, 'a', 0, 0, 0,
        0, 0, 0, 2, 'b', 'c', 0, 0,
    };
    static const uint8_t expected_property[] = {
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0, 0, 0, 0, 2, 'w', 'w', 0, 0,
    };
    static const uint8_t expected_property_list[] = {
        0, 0, 0, 2,
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0, 0, 0, 0, 2, 'w', 'w', 0, 0,
        0, 0, 0, 1, 'q', 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'z', 0, 0, 0,
    };
    static const uint8_t expected_id_list[] = {
        0, 0, 0, 2, 0x41, 0x42, 0x43, 0x44,
        0x51, 0x52, 0x53, 0x54,
    };
    static const uint8_t expected_optional_id[] = {
        0, 0, 0, 1, 0x11, 0x12, 0x13, 0x14,
        0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_addr_info[] = {
        0, 0, 0x12, 0x34, 0, 0, 0x56, 0x78,
    };
    static const uint8_t expected_registration[] = {
        0, 0, 0, 1, 'r', 0, 0, 0,
        0, 0, 0x12, 0x34, 0, 0, 0x56, 0x78,
    };
    static const uint8_t expected_clone_args[] = {
        0, 0, 0, 1, 't', 0, 0, 0, 0, 0, 0, 1, 'm', 0, 0, 0,
        0x0a, 0x0b, 0x0c, 0x0d, 0, 0, 0, 1, 'g', 0, 0, 0,
    };
    static const uint8_t expected_bind_args[] = {
        0x0a, 0, 0, 2, 0, 0, 0, 1, 'c', 0, 0, 0,
        0, 0, 0, 1, 's', 0, 0, 0,
    };
    static const uint8_t expected_getregister_ok[] = {
        0, 0, 0, 0, 0, 0, 0x12, 0x34, 0, 0, 0x56, 0x78,
    };
    static const uint8_t expected_listreg_ok[] = {
        0, 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'r', 0, 0, 0, 0, 0, 0x12, 0x34,
        0, 0, 0x56, 0x78,
    };
    static const uint8_t expected_id_result_ok[] = {
        0, 0, 0, 0, 0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08,
    };
    static const uint8_t expected_parent_result_ok[] = {
        0, 0, 0, 0, 0x41, 0x42, 0x43, 0x44,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_children_result_ok[] = {
        0, 0, 0, 0, 0, 0, 0, 1, 0x41, 0x42, 0x43, 0x44,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_list_result_ok[] = {
        0, 0, 0, 0, 0, 0, 0, 1, 0x61, 0x62, 0x63, 0x64,
        0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 1, 'n', 0, 0, 0,
        0, 0, 0, 1, 'o', 0, 0, 0,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_property_list_stuff[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 2,
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0, 0, 0, 0, 2, 'w', 'w', 0, 0,
        0, 0, 0, 1, 'q', 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'z', 0, 0, 0,
    };
    static const uint8_t expected_create_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 0, 0, 0, 1, 'p', 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 1, 'v', 0, 0, 0,
        0x41, 0x42, 0x43, 0x44, 0, 0, 0, 1,
        0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
    };
    static const uint8_t expected_create_stuff[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_property_list_result_ok[] = {
        0, 0, 0, 0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 2,
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0, 0, 0, 0, 2, 'w', 'w', 0, 0,
        0, 0, 0, 1, 'q', 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'z', 0, 0, 0,
    };
    static const uint8_t expected_object[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 0, 0, 0, 1, 'p', 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 1, 'v', 0, 0, 0,
        0x61, 0x62, 0x63, 0x64, 0, 0, 0, 1, 0x71, 0x72, 0x73, 0x74,
    };
    static const uint8_t expected_entry[] = {
        0x61, 0x62, 0x63, 0x64, 0, 0, 0, 1,
        0, 0, 0, 2, 0, 0, 0, 1, 'n', 0, 0, 0,
        0, 0, 0, 1, 'o', 0, 0, 0,
    };
    static const uint8_t expected_entry_list[] = {
        0, 0, 0, 1,
        0x61, 0x62, 0x63, 0x64, 0, 0, 0, 1,
        0, 0, 0, 2, 0, 0, 0, 1, 'n', 0, 0, 0,
        0, 0, 0, 1, 'o', 0, 0, 0,
    };
    static const uint8_t expected_parent_stuff[] = {
        0x41, 0x42, 0x43, 0x44,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_children_stuff[] = {
        0, 0, 0, 1, 0x41, 0x42, 0x43, 0x44,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_entry_stuff[] = {
        0, 0, 0, 1, 0x61, 0x62, 0x63, 0x64, 0, 0, 0, 1,
        0, 0, 0, 2, 0, 0, 0, 1, 'n', 0, 0, 0,
        0, 0, 0, 1, 'o', 0, 0, 0,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_lookup_stuff[] = {
        0, 0, 0, 1, 0x41, 0x42, 0x43, 0x44,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_name_list_stuff[] = {
        0, 0, 0, 2, 0, 0, 0, 1, 'a', 0, 0, 0,
        0, 0, 0, 2, 'b', 'c', 0, 0,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_read_name_stuff[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 'n', 0, 0, 0,
    };
    static const uint8_t expected_binding[] = {
        0, 0, 0, 1, 'r', 0, 0, 0, 0x01, 0x02, 0x03, 0x04,
    };
    static const uint8_t expected_list_all_stuff[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 0, 0, 0, 2,
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0, 0, 0, 0, 2, 'w', 'w', 0, 0,
        0, 0, 0, 1, 'q', 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'z', 0, 0, 0,
    };
    static const uint8_t expected_create_result_ok[] = {
        0, 0, 0, 0,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_destroy_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_lookup_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 'k', 0, 0, 0, 0, 0, 0, 1, 'v', 0, 0, 0,
    };
    static const uint8_t expected_lookup_result_ok[] = {
        0, 0, 0, 0, 0, 0, 0, 1, 0x41, 0x42, 0x43, 0x44,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_name_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 'n', 0, 0, 0,
    };
    static const uint8_t expected_create_prop_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'v', 0, 0, 0,
        0x41, 0x42, 0x43, 0x44,
    };
    static const uint8_t expected_write_prop_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x51, 0x52, 0x53, 0x54, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0,
        0, 0, 0, 2, 'w', 'w', 0, 0,
    };
    static const uint8_t expected_prop_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x51, 0x52, 0x53, 0x54,
    };
    static const uint8_t expected_name_list_result_ok[] = {
        0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, 'a', 0, 0, 0,
        0, 0, 0, 2, 'b', 'c', 0, 0,
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    };
    static const uint8_t expected_prop_name_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x51, 0x52, 0x53, 0x54, 0, 0, 0, 1, 'q', 0, 0, 0,
    };
    static const uint8_t expected_create_name_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x51, 0x52, 0x53, 0x54, 0, 0, 0, 1, 'n', 0, 0, 0,
        0x61, 0x62, 0x63, 0x64,
    };
    static const uint8_t expected_name_index_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x51, 0x52, 0x53, 0x54, 0x61, 0x62, 0x63, 0x64,
    };
    static const uint8_t expected_write_name_args[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x51, 0x52, 0x53, 0x54, 0x61, 0x62, 0x63, 0x64,
        0, 0, 0, 1, 'o', 0, 0, 0,
    };
    static const uint8_t expected_read_name_result_ok[] = {
        0, 0, 0, 0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 'n', 0, 0, 0,
    };
    static const uint8_t expected_rparent_result_ok[] = {
        0, 0, 0, 0, 0, 0, 0, 1, 'r', 0, 0, 0,
        0x01, 0x02, 0x03, 0x04,
    };
    static const uint8_t expected_object_list[] = {
        0, 0, 0, 1,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 0, 0, 0, 1, 'p', 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 1, 'v', 0, 0, 0,
        0x61, 0x62, 0x63, 0x64, 0, 0, 0, 1, 0x71, 0x72, 0x73, 0x74,
        0, 0, 0, 0,
    };
    static const uint8_t expected_read_all_result_ok[] = {
        0, 0, 0, 0, 0xa1, 0xb2, 0xc3, 0xd4,
        0x51, 0x52, 0x53, 0x54, 0, 0, 0, 1,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 0, 0, 0, 1, 'p', 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 1, 'v', 0, 0, 0,
        0x61, 0x62, 0x63, 0x64, 0, 0, 0, 1, 0x71, 0x72, 0x73, 0x74,
        0, 0, 0, 0,
    };
    static const uint8_t expected_read_all_stuff[] = {
        0xa1, 0xb2, 0xc3, 0xd4, 0x51, 0x52, 0x53, 0x54,
        0, 0, 0, 1,
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1, 0, 0, 0, 1, 'p', 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 1, 'v', 0, 0, 0,
        0x61, 0x62, 0x63, 0x64, 0, 0, 0, 1, 0x71, 0x72, 0x73, 0x74,
        0, 0, 0, 0,
    };
    static const uint8_t expected_property_list_array[] = {
        0, 0, 0, 1,
        0, 0, 0, 2,
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0, 0, 0, 0, 2, 'w', 'w', 0, 0,
        0, 0, 0, 1, 'q', 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'z', 0, 0, 0,
    };
    static const uint8_t expected_list_all_result_ok[] = {
        0, 0, 0, 0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0, 0, 0, 1,
        0, 0, 0, 2,
        0, 0, 0, 1, 'p', 0, 0, 0, 0, 0, 0, 2,
        0, 0, 0, 1, 'v', 0, 0, 0, 0, 0, 0, 2, 'w', 'w', 0, 0,
        0, 0, 0, 1, 'q', 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'z', 0, 0, 0,
    };
    NiId id = { .nii_object = 0x01020304, .nii_instance = 0x05060708 };
    NiId id_b = { .nii_object = 0x11121314, .nii_instance = 0x15161718 };
    NiId id_c = { .nii_object = 0x21222324, .nii_instance = 0x25262728 };
    NiName name = (char *)"x";
    NiName names_ab[] = { (char *)"a", (char *)"bc" };
    NiName property_values[] = { (char *)"v", (char *)"ww" };
    NiName one_value[] = { (char *)"v" };
    NiName write_values[] = { (char *)"v", (char *)"ww" };
    NiName q_values[] = { (char *)"z" };
    NiProperty property = {
        .name = (char *)"p",
        .values = { .count = 2, .values = property_values },
    };
    NiProperty property_one = {
        .name = (char *)"p",
        .values = { .count = 1, .values = one_value },
    };
    NiProperty property_q = {
        .name = (char *)"q",
        .values = { .count = 1, .values = q_values },
    };
    NiProperty properties_pq[] = { property, property_q };
    NiProperty properties_one[] = { property_one };
    NiPropertyList property_list = {
        .count = 2, .properties = properties_pq,
    };
    NiPropertyList property_list_one = {
        .count = 1, .properties = properties_one,
    };
    NiIndex id_values[] = { 0x41424344, 0x51525354 };
    NiIndex lookup_values[] = { 0x41424344 };
    NiIndex parent_child_values[] = { 0x41424344 };
    NiIndex child_values[] = { 0x71727374 };
    NiIdList id_list = { .count = 2, .values = id_values };
    NiIdList lookup_id_list = { .count = 1, .values = lookup_values };
    NiIdList parent_child_list = {
        .count = 1, .values = parent_child_values,
    };
    NiIdList child_list = { .count = 1, .values = child_values };
    NiNameList name_list = { .count = 2, .values = names_ab };
    NiNameList write_name_list = {
        .count = 2, .values = write_values,
    };
    NiBindAddrInfo addr_info = { .udp_port = 0x1234, .tcp_port = 0x5678 };
    NiBindRegistration registration = {
        .tag = (char *)"r", .addrs = { .udp_port = 0x1234,
                                         .tcp_port = 0x5678 },
    };
    NiBindRegistration registrations[] = { registration };
    NiBindCloneArgs clone_args = {
        .tag = (char *)"t",
        .master_name = (char *)"m",
        .master_addr = 0x0a0b0c0d,
        .master_tag = (char *)"g",
    };
    NiBindArgs bind_args = {
        .client_addr = 0x0a000002,
        .client_tag = (char *)"c",
        .server_tag = (char *)"s",
    };
    NiBindGetRegisterResult getregister_ok = {
        .status = NI_OK, .addrs = { .udp_port = 0x1234, .tcp_port = 0x5678 },
    };
    NiBindListRegResult listreg_ok = {
        .status = NI_OK, .count = 1, .registrations = registrations,
    };
    NiIdResult id_result_ok = { .status = NI_OK, .has_id = true, .id = id };
    NiParentStuff parent_stuff = {
        .object_id = 0x41424344, .self_id = id_b,
    };
    NiParentResult parent_result_ok = {
        .status = NI_OK, .stuff = parent_stuff,
    };
    NiChildrenStuff children_stuff = {
        .children = parent_child_list, .self_id = id_b,
    };
    NiChildrenResult children_result_ok = {
        .status = NI_OK, .stuff = children_stuff,
    };
    NiName entry_names[] = { (char *)"n", (char *)"o" };
    NiEntry entry = {
        .id = 0x61626364,
        .has_names = true,
        .names = { .count = 2, .values = entry_names },
    };
    NiEntry entries[] = { entry };
    NiEntryList entry_list = { .count = 1, .entries = entries };
    NiEntryStuff entry_stuff = { .entries = entry_list, .self_id = id_b };
    NiListResult list_result_ok = {
        .status = NI_OK, .stuff = entry_stuff,
    };
    NiPropertyListStuff property_list_stuff = {
        .id = id, .props = property_list,
    };
    NiCreateArgs create_args = {
        .id = id,
        .props = property_list_one,
        .where = 0x41424344,
        .has_target_id = true,
        .target_id = id_c,
    };
    NiPropertyListResult property_list_result_ok = {
        .status = NI_OK, .stuff = property_list_stuff,
    };
    NiCreateStuff create_stuff = { .id = id, .self_id = id_b };
    NiCreateResult create_result_ok = {
        .status = NI_OK, .stuff = create_stuff,
    };
    NiObject object = {
        .id = id,
        .properties = property_list_one,
        .parent = 0x61626364,
        .children = child_list,
    };
    NiObjectNode object_node = { .object = object, .next = NULL };
    NiObjectList object_list = { .head = &object_node, .count = 1 };
    NiLookupStuff lookup_stuff = {
        .idlist = lookup_id_list, .self_id = id_b,
    };
    NiNameListStuff name_list_stuff = {
        .values = name_list, .self_id = id_b,
    };
    NiReadNameStuff read_name_stuff = { .id = id, .name = (char *)"n" };
    NiBinding binding = { .tag = (char *)"r", .addr = 0x01020304 };
    NiPropertyList property_lists[] = { property_list };
    NiPropertyListArray property_list_array = {
        .count = 1, .entries = property_lists,
    };
    NiListAllStuff list_all_stuff = {
        .self_id = id, .entries = property_list_array,
    };
    NiDestroyArgs destroy_args = { .parent_id = id, .self_id = id_b };
    NiLookupArgs lookup_args = {
        .id = id, .key = (char *)"k", .value = (char *)"v",
    };
    NiLookupResult lookup_result_ok = {
        .status = NI_OK, .stuff = lookup_stuff,
    };
    NiNameArgs name_args = { .id = id, .name = (char *)"n" };
    NiCreatePropArgs create_prop_args = {
        .id = id, .prop = property_one, .where = 0x41424344,
    };
    NiWritePropArgs write_prop_args = {
        .id = id, .prop_index = 0x51525354, .values = write_name_list,
    };
    NiPropArgs prop_args = { .id = id, .prop_index = 0x51525354 };
    NiNameListResult name_list_result_ok = {
        .status = NI_OK, .stuff = name_list_stuff,
    };
    NiPropNameArgs prop_name_args = {
        .id = id, .prop_index = 0x51525354, .name = (char *)"q",
    };
    NiCreateNameArgs create_name_args = {
        .id = id, .prop_index = 0x51525354, .name = (char *)"n",
        .where = 0x61626364,
    };
    NiNameIndexArgs name_index_args = {
        .id = id, .prop_index = 0x51525354, .name_index = 0x61626364,
    };
    NiWriteNameArgs write_name_args = {
        .id = id, .prop_index = 0x51525354, .name_index = 0x61626364,
        .name = (char *)"o",
    };
    NiReadNameResult read_name_result_ok = {
        .status = NI_OK, .stuff = read_name_stuff,
    };
    NiRParentResult rparent_result_ok = {
        .status = NI_OK, .binding = binding,
    };
    NiReadAllStuff read_all_stuff = {
        .checksum = 0xa1b2c3d4, .highestid = 0x51525354,
        .list = object_list,
    };
    NiReadAllResult read_all_result_ok = {
        .status = NI_OK, .stuff = read_all_stuff,
    };
    NiListAllResult list_all_result_ok = {
        .status = NI_OK, .stuff = list_all_stuff,
    };
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    uint8_t encoded[sizeof(expected_optional_id)];
    bool present;
    NiId decoded_id;

    NiStatus status = NI_NOUSER;
    NiStatus decoded_status = NI_OK;

    onc_rpc_xdr_reader_init(&reader, expected_status, sizeof(expected_status));
    g_assert_true(ni_xdr_decode_status(&reader, &decoded_status));
    g_assert_cmpint(decoded_status, ==, NI_NOUSER);
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(expected_status));
    g_assert_true(ni_xdr_encode_status(&writer, status));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==,
                     sizeof(expected_status));
    g_assert_cmpmem(encoded, sizeof(expected_status), expected_status,
                    sizeof(expected_status));

    ASSERT_CODEC(id, NiId, ni_id_init, ni_id_clear, &id, expected_id);
    ASSERT_NAME_CODEC(name, expected_name);
    ASSERT_CODEC(name_list, NiNameList, ni_name_list_init, ni_name_list_clear,
                 &name_list, expected_name_list);
    ASSERT_CODEC(property, NiProperty, ni_property_init, ni_property_clear,
                 &property, expected_property);
    ASSERT_CODEC(property_list, NiPropertyList, ni_property_list_init,
                 ni_property_list_clear, &property_list,
                 expected_property_list);
    ASSERT_CODEC(id_list, NiIdList, ni_id_list_init, ni_id_list_clear,
                 &id_list, expected_id_list);

    onc_rpc_xdr_reader_init(&reader, expected_optional_id,
                            sizeof(expected_optional_id));
    g_assert_true(ni_xdr_decode_optional_id(&reader, &decoded_id, &present));
    g_assert_true(present);
    g_assert_cmpuint(decoded_id.nii_object, ==, id_b.nii_object);
    g_assert_cmpuint(decoded_id.nii_instance, ==, id_b.nii_instance);
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    onc_rpc_xdr_writer_init(&writer, encoded, sizeof(encoded));
    g_assert_true(ni_xdr_encode_optional_id(&writer, true, &id_b));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==,
                     sizeof(expected_optional_id));
    g_assert_cmpmem(encoded, sizeof(encoded), expected_optional_id,
                    sizeof(expected_optional_id));

    ASSERT_CODEC(bind_addr_info, NiBindAddrInfo, ni_bind_addr_info_init,
                 ni_bind_addr_info_clear, &addr_info, expected_addr_info);
    ASSERT_CODEC(bind_registration, NiBindRegistration,
                 ni_bind_registration_init, ni_bind_registration_clear,
                 &registration, expected_registration);
    ASSERT_CODEC(bind_clone_args, NiBindCloneArgs, ni_bind_clone_args_init,
                 ni_bind_clone_args_clear, &clone_args, expected_clone_args);
    ASSERT_CODEC(bind_args, NiBindArgs, ni_bind_args_init, ni_bind_args_clear,
                 &bind_args, expected_bind_args);
    ASSERT_CODEC(bind_getregister_result, NiBindGetRegisterResult,
                 ni_bind_getregister_result_init,
                 ni_bind_getregister_result_clear, &getregister_ok,
                 expected_getregister_ok);
    ASSERT_CODEC(bind_listreg_result, NiBindListRegResult,
                 ni_bind_listreg_result_init, ni_bind_listreg_result_clear,
                 &listreg_ok, expected_listreg_ok);

    ASSERT_CODEC(id_result, NiIdResult, ni_id_result_init,
                 ni_id_result_clear, &id_result_ok, expected_id_result_ok);
    ASSERT_CODEC(parent_stuff, NiParentStuff, ni_parent_stuff_init,
                 ni_parent_stuff_clear, &parent_stuff, expected_parent_stuff);
    ASSERT_CODEC(parent_result, NiParentResult, ni_parent_result_init,
                 ni_parent_result_clear, &parent_result_ok,
                 expected_parent_result_ok);
    ASSERT_CODEC(children_stuff, NiChildrenStuff, ni_children_stuff_init,
                 ni_children_stuff_clear, &children_stuff,
                 expected_children_stuff);
    ASSERT_CODEC(children_result, NiChildrenResult, ni_children_result_init,
                 ni_children_result_clear, &children_result_ok,
                 expected_children_result_ok);
    ASSERT_CODEC(entry_stuff, NiEntryStuff, ni_entry_stuff_init,
                 ni_entry_stuff_clear, &entry_stuff, expected_entry_stuff);
    ASSERT_CODEC(list_result, NiListResult, ni_list_result_init,
                 ni_list_result_clear, &list_result_ok,
                 expected_list_result_ok);
    ASSERT_CODEC(property_list_stuff, NiPropertyListStuff,
                 ni_property_list_stuff_init, ni_property_list_stuff_clear,
                 &property_list_stuff, expected_property_list_stuff);
    ASSERT_CODEC(create_args, NiCreateArgs, ni_create_args_init,
                 ni_create_args_clear, &create_args, expected_create_args);
    ASSERT_CODEC(property_list_result, NiPropertyListResult,
                 ni_property_list_result_init, ni_property_list_result_clear,
                 &property_list_result_ok,
                 expected_property_list_result_ok);
    ASSERT_CODEC(create_stuff, NiCreateStuff, ni_create_stuff_init,
                 ni_create_stuff_clear, &create_stuff, expected_create_stuff);
    ASSERT_CODEC(object, NiObject, ni_object_init, ni_object_clear, &object,
                 expected_object);
    ASSERT_CODEC(entry, NiEntry, ni_entry_init, ni_entry_clear, &entry,
                 expected_entry);
    ASSERT_CODEC(entry_list, NiEntryList, ni_entry_list_init,
                 ni_entry_list_clear, &entry_list, expected_entry_list);
    ASSERT_CODEC(lookup_stuff, NiLookupStuff, ni_lookup_stuff_init,
                 ni_lookup_stuff_clear, &lookup_stuff, expected_lookup_stuff);
    ASSERT_CODEC(name_list_stuff, NiNameListStuff, ni_name_list_stuff_init,
                 ni_name_list_stuff_clear, &name_list_stuff,
                 expected_name_list_stuff);
    ASSERT_CODEC(read_name_stuff, NiReadNameStuff, ni_read_name_stuff_init,
                 ni_read_name_stuff_clear, &read_name_stuff,
                 expected_read_name_stuff);
    ASSERT_CODEC(binding, NiBinding, ni_binding_init, ni_binding_clear,
                 &binding, expected_binding);
    ASSERT_CODEC(list_all_stuff, NiListAllStuff, ni_list_all_stuff_init,
                 ni_list_all_stuff_clear, &list_all_stuff,
                 expected_list_all_stuff);
    ASSERT_CODEC(create_result, NiCreateResult, ni_create_result_init,
                 ni_create_result_clear, &create_result_ok,
                 expected_create_result_ok);
    ASSERT_CODEC(destroy_args, NiDestroyArgs, ni_destroy_args_init,
                 ni_destroy_args_clear, &destroy_args, expected_destroy_args);
    ASSERT_CODEC(lookup_args, NiLookupArgs, ni_lookup_args_init,
                 ni_lookup_args_clear, &lookup_args, expected_lookup_args);
    ASSERT_CODEC(lookup_result, NiLookupResult, ni_lookup_result_init,
                 ni_lookup_result_clear, &lookup_result_ok,
                 expected_lookup_result_ok);
    ASSERT_CODEC(name_args, NiNameArgs, ni_name_args_init, ni_name_args_clear,
                 &name_args, expected_name_args);
    ASSERT_CODEC(create_prop_args, NiCreatePropArgs, ni_create_prop_args_init,
                 ni_create_prop_args_clear, &create_prop_args,
                 expected_create_prop_args);
    ASSERT_CODEC(write_prop_args, NiWritePropArgs, ni_write_prop_args_init,
                 ni_write_prop_args_clear, &write_prop_args,
                 expected_write_prop_args);
    ASSERT_CODEC(prop_args, NiPropArgs, ni_prop_args_init, ni_prop_args_clear,
                 &prop_args, expected_prop_args);
    ASSERT_CODEC(name_list_result, NiNameListResult, ni_name_list_result_init,
                 ni_name_list_result_clear, &name_list_result_ok,
                 expected_name_list_result_ok);
    ASSERT_CODEC(prop_name_args, NiPropNameArgs, ni_prop_name_args_init,
                 ni_prop_name_args_clear, &prop_name_args,
                 expected_prop_name_args);
    ASSERT_CODEC(create_name_args, NiCreateNameArgs,
                 ni_create_name_args_init, ni_create_name_args_clear,
                 &create_name_args, expected_create_name_args);
    ASSERT_CODEC(name_index_args, NiNameIndexArgs, ni_name_index_args_init,
                 ni_name_index_args_clear, &name_index_args,
                 expected_name_index_args);
    ASSERT_CODEC(write_name_args, NiWriteNameArgs, ni_write_name_args_init,
                 ni_write_name_args_clear, &write_name_args,
                 expected_write_name_args);
    ASSERT_CODEC(read_name_result, NiReadNameResult,
                 ni_read_name_result_init, ni_read_name_result_clear,
                 &read_name_result_ok, expected_read_name_result_ok);
    ASSERT_CODEC(rparent_result, NiRParentResult, ni_rparent_result_init,
                 ni_rparent_result_clear, &rparent_result_ok,
                 expected_rparent_result_ok);
    ASSERT_CODEC(object_list, NiObjectList, ni_object_list_init,
                 ni_object_list_clear, &object_list, expected_object_list);
    ASSERT_CODEC(read_all_stuff, NiReadAllStuff, ni_read_all_stuff_init,
                 ni_read_all_stuff_clear, &read_all_stuff,
                 expected_read_all_stuff);
    ASSERT_CODEC(read_all_result, NiReadAllResult, ni_read_all_result_init,
                 ni_read_all_result_clear, &read_all_result_ok,
                 expected_read_all_result_ok);
    ASSERT_CODEC(property_list_array, NiPropertyListArray,
                 ni_property_list_array_init, ni_property_list_array_clear,
                 &property_list_array, expected_property_list_array);
    ASSERT_CODEC(list_all_result, NiListAllResult, ni_list_all_result_init,
                 ni_list_all_result_clear, &list_all_result_ok,
                 expected_list_all_result_ok);

    g_assert_cmpuint(sizeof(expected_id), ==, 8);
    g_assert_cmpuint(sizeof(expected_name), ==, 8);
}

#define ASSERT_UNION_FAILURE(name, type, init_fn, clear_fn)                 \
    do {                                                                    \
        static const uint8_t expected[] = { 0, 0, 0, 1 };                  \
        type encoded_value;                                                 \
        type decoded_value;                                                  \
        OncRpcXdrReader union_reader;                                       \
        OncRpcXdrWriter union_writer;                                       \
        uint8_t union_encoded[sizeof(expected)] = { 0 };                    \
        init_fn(&encoded_value);                                             \
        encoded_value.status = NI_BADID;                                    \
        init_fn(&decoded_value);                                             \
        onc_rpc_xdr_writer_init(&union_writer, union_encoded,               \
                                sizeof(union_encoded));                      \
        g_assert_true(ni_xdr_encode_##name(&union_writer, &encoded_value));  \
        g_assert_cmpuint(onc_rpc_xdr_writer_size(&union_writer), ==,          \
                         sizeof(expected));                                  \
        g_assert_cmpmem(union_encoded, sizeof(union_encoded), expected,      \
                        sizeof(expected));                                  \
        onc_rpc_xdr_reader_init(&union_reader, expected, sizeof(expected));  \
        g_assert_true(ni_xdr_decode_##name(&union_reader, &decoded_value));  \
        g_assert_cmpint(decoded_value.status, ==, NI_BADID);                \
        g_assert_true(onc_rpc_xdr_reader_empty(&union_reader));              \
        clear_fn(&encoded_value);                                            \
        clear_fn(&decoded_value);                                            \
    } while (0)

static void test_union_failure_arms(void)
{
    ASSERT_UNION_FAILURE(bind_getregister_result,
                         NiBindGetRegisterResult,
                         ni_bind_getregister_result_init,
                         ni_bind_getregister_result_clear);
    ASSERT_UNION_FAILURE(bind_listreg_result, NiBindListRegResult,
                         ni_bind_listreg_result_init,
                         ni_bind_listreg_result_clear);
    ASSERT_UNION_FAILURE(id_result, NiIdResult, ni_id_result_init,
                         ni_id_result_clear);
    ASSERT_UNION_FAILURE(parent_result, NiParentResult, ni_parent_result_init,
                         ni_parent_result_clear);
    ASSERT_UNION_FAILURE(children_result, NiChildrenResult,
                         ni_children_result_init, ni_children_result_clear);
    ASSERT_UNION_FAILURE(list_result, NiListResult, ni_list_result_init,
                         ni_list_result_clear);
    ASSERT_UNION_FAILURE(property_list_result, NiPropertyListResult,
                         ni_property_list_result_init,
                         ni_property_list_result_clear);
    ASSERT_UNION_FAILURE(create_result, NiCreateResult, ni_create_result_init,
                         ni_create_result_clear);
    ASSERT_UNION_FAILURE(lookup_result, NiLookupResult,
                         ni_lookup_result_init, ni_lookup_result_clear);
    ASSERT_UNION_FAILURE(name_list_result, NiNameListResult,
                         ni_name_list_result_init, ni_name_list_result_clear);
    ASSERT_UNION_FAILURE(read_name_result, NiReadNameResult,
                         ni_read_name_result_init, ni_read_name_result_clear);
    ASSERT_UNION_FAILURE(rparent_result, NiRParentResult,
                         ni_rparent_result_init, ni_rparent_result_clear);
    ASSERT_UNION_FAILURE(read_all_result, NiReadAllResult,
                         ni_read_all_result_init, ni_read_all_result_clear);
    ASSERT_UNION_FAILURE(list_all_result, NiListAllResult,
                         ni_list_all_result_init, ni_list_all_result_clear);
}

#undef ASSERT_UNION_FAILURE
#undef ASSERT_NAME_CODEC
#undef ASSERT_CODEC

static void test_pointer_presence_and_empty_lists(void)
{
    static const uint8_t no_target[] = {
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0,
    };
    static const uint8_t with_target[] = {
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 2,
    };
    static const uint8_t empty_idlist[] = { 0, 0, 0, 0 };
    NiCreateArgs args;
    NiIdList ids;
    OncRpcXdrReader reader;

    ni_create_args_init(&args);
    onc_rpc_xdr_reader_init(&reader, no_target, sizeof(no_target));
    g_assert_true(ni_xdr_decode_create_args(&reader, &args));
    g_assert_false(args.has_target_id);
    g_assert_true(onc_rpc_xdr_reader_empty(&reader));
    ni_create_args_clear(&args);

    ni_create_args_init(&args);
    onc_rpc_xdr_reader_init(&reader, with_target, sizeof(with_target));
    g_assert_true(ni_xdr_decode_create_args(&reader, &args));
    g_assert_true(args.has_target_id);
    g_assert_cmpuint(args.target_id.nii_instance, ==, 2);
    ni_create_args_clear(&args);

    ni_id_list_init(&ids);
    onc_rpc_xdr_reader_init(&reader, empty_idlist, sizeof(empty_idlist));
    g_assert_true(ni_xdr_decode_id_list(&reader, &ids));
    g_assert_cmpuint(ids.count, ==, 0);
    g_assert_null(ids.values);
    ni_id_list_clear(&ids);
}

enum {
    CONTRACT_NIBIND_MAXREGS = 32U,
    CONTRACT_NAME_MAXLEN = 65535U,
    CONTRACT_NAMELIST_MAXLEN = 65535U,
    CONTRACT_PROPLIST_MAXLEN = 65535U,
    CONTRACT_IDLIST_MAXLEN = 1048576U,
    CONTRACT_OBJECT_LIST_MAXLEN = 4096U,
};

static void test_protocol_contract(void)
{
    static const struct {
        uint32_t actual;
        uint32_t expected;
    } programs[] = {
        { NIBIND_PROG, 200100001U },
        { NI_PROG, 200100000U },
    };
    static const struct {
        uint32_t actual;
        uint32_t expected;
    } versions[] = {
        { NIBIND_VERS, 1U },
        { NI_VERS, 2U },
    };
    static const struct {
        uint32_t actual;
        uint32_t expected;
    } ports[] = {
        { NIBIND_UDP_PORT, 659U },
        { NIBIND_TCP_PORT, 661U },
        { NI_UDP_PORT, 660U },
        { NI_TCP_PORT, 662U },
    };
    static const struct {
        uint32_t actual;
        uint32_t expected;
    } bind_procedures[] = {
        { NIBIND_PING, 0U },
        { NIBIND_REGISTER, 1U },
        { NIBIND_UNREGISTER, 2U },
        { NIBIND_GETREGISTER, 3U },
        { NIBIND_LISTREG, 4U },
        { NIBIND_CREATEMASTER, 5U },
        { NIBIND_CREATECLONE, 6U },
        { NIBIND_DESTROYDOMAIN, 7U },
        { NIBIND_BIND, 8U },
    };
    static const struct {
        uint32_t actual;
        uint32_t expected;
    } ni_procedures[] = {
        { NI_PING, 0U },
        { NI_STATISTICS, 1U },
        { NI_ROOT, 2U },
        { NI_SELF, 3U },
        { NI_PARENT, 4U },
        { NI_CREATE, 5U },
        { NI_DESTROY, 6U },
        { NI_READ, 7U },
        { NI_WRITE, 8U },
        { NI_CHILDREN, 9U },
        { NI_LOOKUP, 10U },
        { NI_LIST, 11U },
        { NI_CREATEPROP, 12U },
        { NI_DESTROYPROP, 13U },
        { NI_READPROP, 14U },
        { NI_WRITEPROP, 15U },
        { NI_RENAMEPROP, 16U },
        { NI_LISTPROPS, 17U },
        { NI_CREATENAME, 18U },
        { NI_DESTROYNAME, 19U },
        { NI_READNAME, 20U },
        { NI_WRITENAME, 21U },
        { NI_RPARENT, 22U },
        { NI_LISTALL, 23U },
        { NI_BIND, 24U },
        { NI_READALL, 25U },
        { NI_CRASHED, 26U },
        { NI_RESYNC, 27U },
        { NI_LOOKUPREAD, 28U },
    };
    static const struct {
        NiStatus actual;
        int32_t expected;
    } statuses[] = {
        { NI_OK, 0 },
        { NI_BADID, 1 },
        { NI_STALE, 2 },
        { NI_NOSPACE, 3 },
        { NI_PERM, 4 },
        { NI_NODIR, 5 },
        { NI_NOPROP, 6 },
        { NI_NONAME, 7 },
        { NI_NOTEMPTY, 8 },
        { NI_UNRELATED, 9 },
        { NI_SERIAL, 10 },
        { NI_NETROOT, 11 },
        { NI_NORESPONSE, 12 },
        { NI_RDONLY, 13 },
        { NI_SYSTEMERR, 14 },
        { NI_ALIVE, 15 },
        { NI_NOTMASTER, 16 },
        { NI_CANTFINDADDRESS, 17 },
        { NI_DUPTAG, 18 },
        { NI_NOTAG, 19 },
        { NI_AUTHERROR, 20 },
        { NI_NOUSER, 21 },
        { NI_FAILED, 9999 },
    };

    for (size_t i = 0; i < ARRAY_SIZE(programs); i++) {
        g_assert_cmpuint(programs[i].actual, ==, programs[i].expected);
    }
    for (size_t i = 0; i < ARRAY_SIZE(versions); i++) {
        g_assert_cmpuint(versions[i].actual, ==, versions[i].expected);
    }
    for (size_t i = 0; i < ARRAY_SIZE(ports); i++) {
        g_assert_cmpuint(ports[i].actual, ==, ports[i].expected);
    }
    for (size_t i = 0; i < ARRAY_SIZE(bind_procedures); i++) {
        g_assert_cmpuint(bind_procedures[i].actual, ==,
                         bind_procedures[i].expected);
    }
    for (size_t i = 0; i < ARRAY_SIZE(ni_procedures); i++) {
        g_assert_cmpuint(ni_procedures[i].actual, ==,
                         ni_procedures[i].expected);
    }
    for (size_t i = 0; i < ARRAY_SIZE(statuses); i++) {
        g_assert_cmpint(statuses[i].actual, ==, statuses[i].expected);
    }
}

static void test_name_maximum(void)
{
    g_autofree char *name = g_malloc(CONTRACT_NAME_MAXLEN + 2U);
    g_autofree char *unterminated =
        g_malloc(CONTRACT_NAME_MAXLEN + 1U);
    g_autofree uint8_t *wire = g_malloc(65540U);
    g_autofree uint8_t *over_wire = g_malloc(65540U);
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;
    NiName decoded;

    ni_name_init(&decoded);
    g_assert_cmpuint(NI_NAME_MAXLEN, ==, CONTRACT_NAME_MAXLEN);
    over_wire[0] = 0;
    over_wire[1] = 1;
    over_wire[2] = 0;
    over_wire[3] = 0;
    memset(over_wire + 4, 'x', 65536U);
    memset(name, 'x', CONTRACT_NAME_MAXLEN);
    name[CONTRACT_NAME_MAXLEN] = '\0';
    memset(unterminated, 'x', CONTRACT_NAME_MAXLEN + 1U);
    onc_rpc_xdr_writer_init(&writer, wire, 65540U);
    g_assert_true(ni_xdr_encode_name(&writer, name));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 65540U);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_name(&reader, &decoded));
    g_assert_cmpuint(strlen(decoded), ==, CONTRACT_NAME_MAXLEN);
    ni_name_clear(&decoded);

    name[CONTRACT_NAME_MAXLEN] = 'x';
    name[CONTRACT_NAME_MAXLEN + 1U] = '\0';
    onc_rpc_xdr_writer_init(&writer, wire, 65540U);
    g_assert_false(ni_xdr_encode_name(&writer, name));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);

    /*
     * Bounded scanning must reject an unterminated buffer without reading
     * past the historical name bound.
     */
    onc_rpc_xdr_writer_init(&writer, wire, 65540U);
    g_assert_false(ni_xdr_encode_name(&writer, unterminated));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);

    onc_rpc_xdr_reader_init(&reader, over_wire, 65540U);
    g_assert_false(ni_xdr_decode_name(&reader, &decoded));
    g_assert_null(decoded);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 65540U);
}

static void test_name_list_maximum(void)
{
    g_autofree NiName *values =
        g_new0(NiName, CONTRACT_NAMELIST_MAXLEN);
    g_autofree uint8_t *wire = g_malloc(262144U);
    g_autofree uint8_t *over_wire = g_malloc(262148U);
    NiNameList list = {
        .count = CONTRACT_NAMELIST_MAXLEN, .values = values,
    };
    NiNameList decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;

    g_assert_cmpuint(NI_NAMELIST_MAXLEN, ==, CONTRACT_NAMELIST_MAXLEN);
    over_wire[0] = 0;
    over_wire[1] = 1;
    over_wire[2] = 0;
    over_wire[3] = 0;
    memset(over_wire + 4, 0, 262144U);
    for (size_t i = 0; i < CONTRACT_NAMELIST_MAXLEN; i++) {
        values[i] = (char *)"";
    }
    onc_rpc_xdr_writer_init(&writer, wire, 262144U);
    g_assert_true(ni_xdr_encode_name_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 262144U);
    ni_name_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_name_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, CONTRACT_NAMELIST_MAXLEN);
    ni_name_list_clear(&decoded);

    list.count = CONTRACT_NAMELIST_MAXLEN + 1U;
    onc_rpc_xdr_writer_init(&writer, wire, 262144U);
    g_assert_false(ni_xdr_encode_name_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);
    ni_name_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, over_wire, 262148U);
    g_assert_false(ni_xdr_decode_name_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, 0U);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 262148U);
    ni_name_list_clear(&decoded);
}

static void test_property_list_maximum(void)
{
    g_autofree NiProperty *properties =
        g_new0(NiProperty, CONTRACT_PROPLIST_MAXLEN);
    g_autofree uint8_t *wire = g_malloc(524284U);
    g_autofree uint8_t *over_wire = g_malloc(524292U);
    NiPropertyList list = {
        .count = CONTRACT_PROPLIST_MAXLEN, .properties = properties,
    };
    NiPropertyList decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;

    g_assert_cmpuint(NI_PROPLIST_MAXLEN, ==, CONTRACT_PROPLIST_MAXLEN);
    over_wire[0] = 0;
    over_wire[1] = 1;
    over_wire[2] = 0;
    over_wire[3] = 0;
    memset(over_wire + 4, 0, 524288U);
    for (size_t i = 0; i < CONTRACT_PROPLIST_MAXLEN; i++) {
        properties[i].name = (char *)"";
    }
    onc_rpc_xdr_writer_init(&writer, wire, 524284U);
    g_assert_true(ni_xdr_encode_property_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 524284U);
    ni_property_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_property_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, CONTRACT_PROPLIST_MAXLEN);
    ni_property_list_clear(&decoded);

    list.count = CONTRACT_PROPLIST_MAXLEN + 1U;
    onc_rpc_xdr_writer_init(&writer, wire, 524284U);
    g_assert_false(ni_xdr_encode_property_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);
    ni_property_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, over_wire, 524292U);
    g_assert_false(ni_xdr_decode_property_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, 0U);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 524292U);
    ni_property_list_clear(&decoded);
}

static void test_id_list_maximum(void)
{
    g_autofree NiIndex *values =
        g_new0(NiIndex, CONTRACT_IDLIST_MAXLEN);
    g_autofree uint8_t *wire = g_malloc(4194308U);
    g_autofree uint8_t *over_wire = g_malloc(4194312U);
    NiIdList list = {
        .count = CONTRACT_IDLIST_MAXLEN, .values = values,
    };
    NiIdList decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;

    g_assert_cmpuint(NI_IDLIST_MAXLEN, ==, CONTRACT_IDLIST_MAXLEN);
    over_wire[0] = 0;
    over_wire[1] = 0x10;
    over_wire[2] = 0;
    over_wire[3] = 1;
    memset(over_wire + 4, 0, 4194308U);
    onc_rpc_xdr_writer_init(&writer, wire, 4194308U);
    g_assert_true(ni_xdr_encode_id_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 4194308U);
    ni_id_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_id_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, CONTRACT_IDLIST_MAXLEN);
    ni_id_list_clear(&decoded);

    list.count = CONTRACT_IDLIST_MAXLEN + 1U;
    onc_rpc_xdr_writer_init(&writer, wire, 4194308U);
    g_assert_false(ni_xdr_encode_id_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);
    ni_id_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, over_wire, 4194312U);
    g_assert_false(ni_xdr_decode_id_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, 0U);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 4194312U);
    ni_id_list_clear(&decoded);
}

static void test_bind_registration_maximum(void)
{
    g_autofree NiBindRegistration *registrations =
        g_new0(NiBindRegistration, CONTRACT_NIBIND_MAXREGS);
    g_autofree uint8_t *wire = g_malloc(392U);
    g_autofree uint8_t *over_wire = g_malloc(404U);
    NiBindListRegResult list = {
        .status = NI_OK,
        .count = CONTRACT_NIBIND_MAXREGS,
        .registrations = registrations,
    };
    NiBindListRegResult decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;

    g_assert_cmpuint(NIBIND_MAXREGS, ==, CONTRACT_NIBIND_MAXREGS);
    memset(over_wire, 0, 404U);
    over_wire[7] = 33;
    for (size_t i = 0; i < CONTRACT_NIBIND_MAXREGS; i++) {
        registrations[i].tag = (char *)"";
    }
    onc_rpc_xdr_writer_init(&writer, wire, 392U);
    g_assert_true(ni_xdr_encode_bind_listreg_result(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 392U);
    ni_bind_listreg_result_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_bind_listreg_result(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, CONTRACT_NIBIND_MAXREGS);
    ni_bind_listreg_result_clear(&decoded);

    list.count = CONTRACT_NIBIND_MAXREGS + 1U;
    onc_rpc_xdr_writer_init(&writer, wire, 392U);
    g_assert_false(ni_xdr_encode_bind_listreg_result(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);
    ni_bind_listreg_result_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, over_wire, 404U);
    g_assert_false(ni_xdr_decode_bind_listreg_result(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, 0U);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 404U);
    ni_bind_listreg_result_clear(&decoded);
}

static void test_entry_list_maximum(void)
{
    g_autofree NiEntry *entries =
        g_new0(NiEntry, CONTRACT_IDLIST_MAXLEN);
    g_autofree uint8_t *wire = g_malloc(8388612U);
    g_autofree uint8_t *over_wire = g_malloc(8388620U);
    NiEntryList list = {
        .count = CONTRACT_IDLIST_MAXLEN, .entries = entries,
    };
    NiEntryList decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;

    over_wire[0] = 0;
    over_wire[1] = 0x10;
    over_wire[2] = 0;
    over_wire[3] = 1;
    memset(over_wire + 4, 0, 8388616U);
    for (size_t i = 0; i < CONTRACT_IDLIST_MAXLEN; i++) {
        entries[i].has_names = false;
    }
    onc_rpc_xdr_writer_init(&writer, wire, 8388612U);
    g_assert_true(ni_xdr_encode_entry_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 8388612U);
    ni_entry_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_entry_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, CONTRACT_IDLIST_MAXLEN);
    ni_entry_list_clear(&decoded);

    list.count = CONTRACT_IDLIST_MAXLEN + 1U;
    onc_rpc_xdr_writer_init(&writer, wire, 8388612U);
    g_assert_false(ni_xdr_encode_entry_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);
    ni_entry_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, over_wire, 8388620U);
    g_assert_false(ni_xdr_decode_entry_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, 0U);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 8388620U);
    ni_entry_list_clear(&decoded);
}

static void test_property_list_array_maximum(void)
{
    g_autofree NiPropertyList *entries =
        g_new0(NiPropertyList, CONTRACT_IDLIST_MAXLEN);
    g_autofree uint8_t *wire = g_malloc(4194308U);
    g_autofree uint8_t *over_wire = g_malloc(4194312U);
    NiPropertyListArray array = {
        .count = CONTRACT_IDLIST_MAXLEN, .entries = entries,
    };
    NiPropertyListArray decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;

    over_wire[0] = 0;
    over_wire[1] = 0x10;
    over_wire[2] = 0;
    over_wire[3] = 1;
    memset(over_wire + 4, 0, 4194308U);
    onc_rpc_xdr_writer_init(&writer, wire, 4194308U);
    g_assert_true(ni_xdr_encode_property_list_array(&writer, &array));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 4194308U);
    ni_property_list_array_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_property_list_array(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, CONTRACT_IDLIST_MAXLEN);
    ni_property_list_array_clear(&decoded);

    array.count = CONTRACT_IDLIST_MAXLEN + 1U;
    onc_rpc_xdr_writer_init(&writer, wire, 4194308U);
    g_assert_false(ni_xdr_encode_property_list_array(&writer, &array));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);
    ni_property_list_array_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, over_wire, 4194312U);
    g_assert_false(ni_xdr_decode_property_list_array(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, 0U);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 4194312U);
    ni_property_list_array_clear(&decoded);
}

static void test_object_list_maximum(void)
{
    g_autofree NiObjectNode *nodes =
        g_new0(NiObjectNode, CONTRACT_OBJECT_LIST_MAXLEN);
    g_autofree NiObjectNode *over_nodes =
        g_new0(NiObjectNode, CONTRACT_OBJECT_LIST_MAXLEN + 1U);
    g_autofree uint8_t *wire = g_malloc(98308U);
    g_autofree uint8_t *over_wire = g_malloc(98332U);
    NiObjectList list = {
        .head = nodes, .count = CONTRACT_OBJECT_LIST_MAXLEN,
    };
    NiObjectList over_list = {
        .head = over_nodes, .count = CONTRACT_OBJECT_LIST_MAXLEN + 1U,
    };
    NiObjectList decoded;
    OncRpcXdrReader reader;
    OncRpcXdrWriter writer;

    g_assert_cmpuint(NI_OBJECT_LIST_MAXLEN, ==,
                     CONTRACT_OBJECT_LIST_MAXLEN);
    for (size_t i = 0; i < CONTRACT_OBJECT_LIST_MAXLEN; i++) {
        nodes[i].next = i + 1U < CONTRACT_OBJECT_LIST_MAXLEN ?
                        &nodes[i + 1U] : NULL;
    }
    onc_rpc_xdr_writer_init(&writer, wire, 98308U);
    g_assert_true(ni_xdr_encode_object_list(&writer, &list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 98308U);
    ni_object_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_true(ni_xdr_decode_object_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, CONTRACT_OBJECT_LIST_MAXLEN);
    ni_object_list_clear(&decoded);

    for (size_t i = 0; i < CONTRACT_OBJECT_LIST_MAXLEN + 1U; i++) {
        over_nodes[i].next = i + 1U < CONTRACT_OBJECT_LIST_MAXLEN + 1U ?
                             &over_nodes[i + 1U] : NULL;
    }
    onc_rpc_xdr_writer_init(&writer, wire, 98308U);
    g_assert_false(ni_xdr_encode_object_list(&writer, &over_list));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 0U);

    /* Spell a max+1 linked-list wire image without the codec under test. */
    onc_rpc_xdr_writer_init(&writer, over_wire, 98332U);
    for (size_t i = 0; i < CONTRACT_OBJECT_LIST_MAXLEN + 1U; i++) {
        g_assert_true(onc_rpc_xdr_put_bool(&writer, true));
        for (size_t field = 0; field < 5U; field++) {
            g_assert_true(onc_rpc_xdr_put_u32(&writer, 0));
        }
    }
    g_assert_true(onc_rpc_xdr_put_bool(&writer, false));
    g_assert_cmpuint(onc_rpc_xdr_writer_size(&writer), ==, 98332U);
    ni_object_list_init(&decoded);
    onc_rpc_xdr_reader_init(&reader, over_wire,
                            onc_rpc_xdr_writer_size(&writer));
    g_assert_false(ni_xdr_decode_object_list(&reader, &decoded));
    g_assert_cmpuint(decoded.count, ==, 0U);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==, 98332U);
    ni_object_list_clear(&decoded);
}

static void test_protocol_maxima(void)
{
    test_protocol_contract();
    test_name_maximum();
    test_name_list_maximum();
    test_property_list_maximum();
    test_id_list_maximum();
    test_bind_registration_maximum();
    test_entry_list_maximum();
    test_property_list_array_maximum();
    test_object_list_maximum();
}

static void test_object_list_policy(void)
{
    g_assert_cmpuint(NI_OBJECT_LIST_MAXLEN, ==, 4096U);
}

static void test_decode_rejections_and_reuse(void)
{
    static const uint8_t valid_name[] = {
        0, 0, 0, 1, 'x', 0, 0, 0,
    };
    static const uint8_t truncated_name[] = {
        0, 0, 0, 1, 'x', 0, 0,
    };
    static const uint8_t extra_name[] = {
        0, 0, 0, 1, 'x', 0, 0, 0, 0, 0, 0, 0,
    };
    static const uint8_t malformed_padding[] = {
        0, 0, 0, 1, 'x', 0xad, 0xbe, 0xef,
    };
    static const uint8_t noncanonical_bool[] = { 0, 0, 0, 2 };
    static const uint8_t unsupported_status[] = { 0, 0, 0, 22 };
    static const uint8_t truncated_registration[] = {
        0, 0, 0, 1, 'x', 0, 0, 0, 0, 0, 0, 0,
    };
    static const uint8_t truncated_listreg[] = {
        0, 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 1, 'x', 0, 0, 0, 0, 0, 0, 0,
    };
    OncRpcXdrReader reader;
    NiName name;
    NiBindRegistration registration;
    NiBindListRegResult listreg;
    NiPropertyListResult result;

    ni_name_init(&name);
    onc_rpc_xdr_reader_init(&reader, valid_name, sizeof(valid_name));
    g_assert_true(ni_xdr_decode_name(&reader, &name));
    g_assert_cmpstr(name, ==, "x");
    ni_name_clear(&name);

    onc_rpc_xdr_reader_init(&reader, truncated_name, sizeof(truncated_name));
    g_assert_false(ni_xdr_decode_name(&reader, &name));
    g_assert_null(name);
    g_assert_cmpuint(onc_rpc_xdr_reader_remaining(&reader), ==,
                     sizeof(truncated_name));

    onc_rpc_xdr_reader_init(&reader, extra_name, sizeof(extra_name));
    g_assert_false(ni_xdr_decode_name(&reader, &name));
    g_assert_null(name);

    onc_rpc_xdr_reader_init(&reader, malformed_padding,
                            sizeof(malformed_padding));
    g_assert_false(ni_xdr_decode_name(&reader, &name));
    g_assert_null(name);

    onc_rpc_xdr_reader_init(&reader, noncanonical_bool,
                            sizeof(noncanonical_bool));
    g_assert_false(ni_xdr_decode_optional_id(&reader, &(NiId){ 0 }, NULL));

    onc_rpc_xdr_reader_init(&reader, unsupported_status,
                            sizeof(unsupported_status));
    g_assert_false(ni_xdr_decode_status(&reader, &(NiStatus){ 0 }));

    ni_bind_registration_init(&registration);
    onc_rpc_xdr_reader_init(&reader, truncated_registration,
                            sizeof(truncated_registration));
    g_assert_false(ni_xdr_decode_bind_registration(&reader, &registration));
    g_assert_null(registration.tag);

    ni_bind_listreg_result_init(&listreg);
    onc_rpc_xdr_reader_init(&reader, truncated_listreg,
                            sizeof(truncated_listreg));
    g_assert_false(ni_xdr_decode_bind_listreg_result(&reader, &listreg));
    g_assert_cmpuint(listreg.count, ==, 0);
    g_assert_null(listreg.registrations);
    ni_bind_registration_clear(&registration);
    ni_bind_listreg_result_clear(&listreg);

    ni_property_list_result_init(&result);
    /* A failed decode after a successful allocation must be reusable. */
    {
        static const uint8_t malformed_result[] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
            0, 0, 0, 1, 0, 0, 0, 4, 'n', 'a', 'm', 'e',
            0, 0, 0, 1, 0, 0, 0,
        };
        onc_rpc_xdr_reader_init(&reader, malformed_result,
                                sizeof(malformed_result));
        g_assert_false(ni_xdr_decode_property_list_result(&reader, &result));
        g_assert_cmpuint(result.stuff.props.count, ==, 0);
        g_assert_null(result.stuff.props.properties);
    }
    ni_property_list_result_clear(&result);
}

static void test_decode_transactional_destinations(void)
{
    static const uint8_t valid_name[] = {
        0, 0, 0, 3, 'n', 'e', 'w', 0,
    };
    static const uint8_t truncated_name[] = {
        0, 0, 0, 3, 'n', 'e', 'w',
    };
    static const uint8_t trailing_name[] = {
        0, 0, 0, 3, 'n', 'e', 'w', 0, 0, 0, 0, 0,
    };
    static const uint8_t valid_result[] = {
        0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 2,
        0, 0, 0, 1,
        0, 0, 0, 3, 'o', 'l', 'd', 0,
        0, 0, 0, 1,
        0, 0, 0, 3, 'v', 'a', 'l', 0,
    };
    static const uint8_t truncated_result[] = {
        0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 2,
        0, 0, 0, 1,
        0, 0, 0, 3, 'o', 'l', 'd', 0,
        0, 0, 0, 1,
        0, 0, 0, 3, 'v', 'a', 'l',
    };
    static const uint8_t trailing_result[] = {
        0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0, 2,
        0, 0, 0, 1,
        0, 0, 0, 3, 'o', 'l', 'd', 0,
        0, 0, 0, 1,
        0, 0, 0, 3, 'v', 'a', 'l', 0,
        0, 0, 0, 0,
    };
    static const uint8_t replacement_result[] = {
        0, 0, 0, 0,
        0, 0, 0, 3, 0, 0, 0, 4,
        0, 0, 0, 1,
        0, 0, 0, 3, 'n', 'e', 'w', 0,
        0, 0, 0, 1,
        0, 0, 0, 1, 'x', 0, 0, 0,
    };
    OncRpcXdrReader reader;
    OncRpcXdrReader before;
    NiName name;
    NiPropertyListResult result;
    NiName old_name;
    NiName old_value;

    ni_name_init(&name);
    name = g_strdup("keep");

    onc_rpc_xdr_reader_init(&reader, truncated_name, sizeof(truncated_name));
    before = reader;
    g_assert_false(ni_xdr_decode_name(&reader, &name));
    g_assert_cmpstr(name, ==, "keep");
    g_assert_true(reader.cursor == before.cursor);
    g_assert_true(reader.end == before.end);

    onc_rpc_xdr_reader_init(&reader, trailing_name, sizeof(trailing_name));
    before = reader;
    g_assert_false(ni_xdr_decode_name(&reader, &name));
    g_assert_cmpstr(name, ==, "keep");
    g_assert_true(reader.cursor == before.cursor);
    g_assert_true(reader.end == before.end);

    onc_rpc_xdr_reader_init(&reader, valid_name, sizeof(valid_name));
    g_assert_true(ni_xdr_decode_name(&reader, &name));
    g_assert_cmpstr(name, ==, "new");
    ni_name_clear(&name);

    ni_property_list_result_init(&result);
    onc_rpc_xdr_reader_init(&reader, valid_result, sizeof(valid_result));
    g_assert_true(ni_xdr_decode_property_list_result(&reader, &result));
    old_name = result.stuff.props.properties[0].name;
    old_value = result.stuff.props.properties[0].values.values[0];
    g_assert_cmpstr(old_name, ==, "old");
    g_assert_cmpstr(old_value, ==, "val");

    onc_rpc_xdr_reader_init(&reader, truncated_result,
                            sizeof(truncated_result));
    before = reader;
    g_assert_false(ni_xdr_decode_property_list_result(&reader, &result));
    g_assert_true(reader.cursor == before.cursor);
    g_assert_true(reader.end == before.end);
    g_assert_true(result.stuff.props.properties[0].name == old_name);
    g_assert_true(result.stuff.props.properties[0].values.values[0] ==
                  old_value);
    g_assert_cmpstr(result.stuff.props.properties[0].name, ==, "old");
    g_assert_cmpstr(result.stuff.props.properties[0].values.values[0], ==,
                    "val");

    onc_rpc_xdr_reader_init(&reader, trailing_result,
                            sizeof(trailing_result));
    before = reader;
    g_assert_false(ni_xdr_decode_property_list_result(&reader, &result));
    g_assert_true(reader.cursor == before.cursor);
    g_assert_true(reader.end == before.end);
    g_assert_true(result.stuff.props.properties[0].name == old_name);
    g_assert_true(result.stuff.props.properties[0].values.values[0] ==
                  old_value);
    g_assert_cmpstr(result.stuff.props.properties[0].name, ==, "old");
    g_assert_cmpstr(result.stuff.props.properties[0].values.values[0], ==,
                    "val");

    onc_rpc_xdr_reader_init(&reader, replacement_result,
                            sizeof(replacement_result));
    g_assert_true(ni_xdr_decode_property_list_result(&reader, &result));
    g_assert_cmpuint(result.stuff.id.nii_object, ==, 3);
    g_assert_cmpuint(result.stuff.id.nii_instance, ==, 4);
    g_assert_cmpstr(result.stuff.props.properties[0].name, ==, "new");
    g_assert_cmpstr(result.stuff.props.properties[0].values.values[0], ==,
                    "x");
    ni_property_list_result_clear(&result);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/netinfo-xdr/golden/getregister-network",
                    test_getregister_network_golden);
    g_test_add_func("/netinfo-xdr/golden/registration-ports",
                    test_registration_ports_golden);
    g_test_add_func("/netinfo-xdr/golden/bind-arguments",
                    test_bind_arguments_golden);
    g_test_add_func("/netinfo-xdr/golden/root-result",
                    test_root_result_golden);
    g_test_add_func("/netinfo-xdr/golden/lookup-arguments",
                    test_lookup_arguments_golden);
    g_test_add_func("/netinfo-xdr/golden/read-result",
                    test_read_result_golden);
    g_test_add_func("/netinfo-xdr/all-public-codec-shapes",
                    test_all_public_codec_shapes);
    g_test_add_func("/netinfo-xdr/union-failure-arms",
                    test_union_failure_arms);
    g_test_add_func("/netinfo-xdr/pointers-and-empty",
                    test_pointer_presence_and_empty_lists);
    g_test_add_func("/netinfo-xdr/maxima", test_protocol_maxima);
    g_test_add_func("/netinfo-xdr/object-list-policy",
                    test_object_list_policy);
    g_test_add_func("/netinfo-xdr/rejections-and-reuse",
                    test_decode_rejections_and_reuse);
    g_test_add_func("/netinfo-xdr/transactional-destinations",
                    test_decode_transactional_destinations);
    return g_test_run();
}
