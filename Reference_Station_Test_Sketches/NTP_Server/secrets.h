//Your AssistNow token
const char myAssistNowToken[] = "<Your Token>";


// BearSSL Certificate
// https://openslab-osu.github.io/bearssl-certificate-utility/

/* This file is auto-generated by the pycert_bearssl tool.  Do not change it manually.
 * Certificates are BearSSL br_x509_trust_anchor format.  Included certs:
 *
 * Index:    0
 * Label:    Starfield Class 2 Certification Authority
 * Subject:  OU=Starfield Class 2 Certification Authority,O=Starfield Technologies\, Inc.,C=US
 * Domain(s): online-live1.services.u-blox.com
 */

#define TAs_NUM 1

static const unsigned char TA_DN0[] = {
    0x30, 0x68, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13,
    0x02, 0x55, 0x53, 0x31, 0x25, 0x30, 0x23, 0x06, 0x03, 0x55, 0x04, 0x0a,
    0x13, 0x1c, 0x53, 0x74, 0x61, 0x72, 0x66, 0x69, 0x65, 0x6c, 0x64, 0x20,
    0x54, 0x65, 0x63, 0x68, 0x6e, 0x6f, 0x6c, 0x6f, 0x67, 0x69, 0x65, 0x73,
    0x2c, 0x20, 0x49, 0x6e, 0x63, 0x2e, 0x31, 0x32, 0x30, 0x30, 0x06, 0x03,
    0x55, 0x04, 0x0b, 0x13, 0x29, 0x53, 0x74, 0x61, 0x72, 0x66, 0x69, 0x65,
    0x6c, 0x64, 0x20, 0x43, 0x6c, 0x61, 0x73, 0x73, 0x20, 0x32, 0x20, 0x43,
    0x65, 0x72, 0x74, 0x69, 0x66, 0x69, 0x63, 0x61, 0x74, 0x69, 0x6f, 0x6e,
    0x20, 0x41, 0x75, 0x74, 0x68, 0x6f, 0x72, 0x69, 0x74, 0x79,
};

static const unsigned char TA_RSA_N0[] = {
    0xb7, 0x32, 0xc8, 0xfe, 0xe9, 0x71, 0xa6, 0x04, 0x85, 0xad, 0x0c, 0x11,
    0x64, 0xdf, 0xce, 0x4d, 0xef, 0xc8, 0x03, 0x18, 0x87, 0x3f, 0xa1, 0xab,
    0xfb, 0x3c, 0xa6, 0x9f, 0xf0, 0xc3, 0xa1, 0xda, 0xd4, 0xd8, 0x6e, 0x2b,
    0x53, 0x90, 0xfb, 0x24, 0xa4, 0x3e, 0x84, 0xf0, 0x9e, 0xe8, 0x5f, 0xec,
    0xe5, 0x27, 0x44, 0xf5, 0x28, 0xa6, 0x3f, 0x7b, 0xde, 0xe0, 0x2a, 0xf0,
    0xc8, 0xaf, 0x53, 0x2f, 0x9e, 0xca, 0x05, 0x01, 0x93, 0x1e, 0x8f, 0x66,
    0x1c, 0x39, 0xa7, 0x4d, 0xfa, 0x5a, 0xb6, 0x73, 0x04, 0x25, 0x66, 0xeb,
    0x77, 0x7f, 0xe7, 0x59, 0xc6, 0x4a, 0x99, 0x25, 0x14, 0x54, 0xeb, 0x26,
    0xc7, 0xf3, 0x7f, 0x19, 0xd5, 0x30, 0x70, 0x8f, 0xaf, 0xb0, 0x46, 0x2a,
    0xff, 0xad, 0xeb, 0x29, 0xed, 0xd7, 0x9f, 0xaa, 0x04, 0x87, 0xa3, 0xd4,
    0xf9, 0x89, 0xa5, 0x34, 0x5f, 0xdb, 0x43, 0x91, 0x82, 0x36, 0xd9, 0x66,
    0x3c, 0xb1, 0xb8, 0xb9, 0x82, 0xfd, 0x9c, 0x3a, 0x3e, 0x10, 0xc8, 0x3b,
    0xef, 0x06, 0x65, 0x66, 0x7a, 0x9b, 0x19, 0x18, 0x3d, 0xff, 0x71, 0x51,
    0x3c, 0x30, 0x2e, 0x5f, 0xbe, 0x3d, 0x77, 0x73, 0xb2, 0x5d, 0x06, 0x6c,
    0xc3, 0x23, 0x56, 0x9a, 0x2b, 0x85, 0x26, 0x92, 0x1c, 0xa7, 0x02, 0xb3,
    0xe4, 0x3f, 0x0d, 0xaf, 0x08, 0x79, 0x82, 0xb8, 0x36, 0x3d, 0xea, 0x9c,
    0xd3, 0x35, 0xb3, 0xbc, 0x69, 0xca, 0xf5, 0xcc, 0x9d, 0xe8, 0xfd, 0x64,
    0x8d, 0x17, 0x80, 0x33, 0x6e, 0x5e, 0x4a, 0x5d, 0x99, 0xc9, 0x1e, 0x87,
    0xb4, 0x9d, 0x1a, 0xc0, 0xd5, 0x6e, 0x13, 0x35, 0x23, 0x5e, 0xdf, 0x9b,
    0x5f, 0x3d, 0xef, 0xd6, 0xf7, 0x76, 0xc2, 0xea, 0x3e, 0xbb, 0x78, 0x0d,
    0x1c, 0x42, 0x67, 0x6b, 0x04, 0xd8, 0xf8, 0xd6, 0xda, 0x6f, 0x8b, 0xf2,
    0x44, 0xa0, 0x01, 0xab,
};

static const unsigned char TA_RSA_E0[] = {
    0x03,
};

static const br_x509_trust_anchor TAs[] = {
    {
        { (unsigned char *)TA_DN0, sizeof TA_DN0 },
        BR_X509_TA_CA,
        {
            BR_KEYTYPE_RSA,
            { .rsa = {
                (unsigned char *)TA_RSA_N0, sizeof TA_RSA_N0,
                (unsigned char *)TA_RSA_E0, sizeof TA_RSA_E0,
            } }
        }
    },
};
