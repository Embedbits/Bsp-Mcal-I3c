/**
 * \author Mr.Nobody
 * \file BspMain.h
 * \ingroup I3c
 * \brief Entry point of I3c integration test firmware (called by StartUp).
 *
 * StartUp calls BspMain() and gets this header through the interface library
 * BspMain_Lib (created by Tests/IntegrationTests/CMakeLists.txt). BspMain() is
 * implemented by ItTarget_I3c.c.
 */

#ifndef I3C_ITTEST_BSPMAIN_H
#define I3C_ITTEST_BSPMAIN_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* ======================== EXPORTED FUNCTIONS ============================== */

void BspMain( void );

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* I3C_ITTEST_BSPMAIN_H */
