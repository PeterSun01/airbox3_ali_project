#ifndef __HMAC_SHA1_H__
#define __HMAC_SHA1_H__


/*
参数说明：
msg-要加密的数据
msg_len-数据长度
digest-输出加密结果
key-加密因子
key_len-加密因子长度
*/
extern void aliyun_iot_common_hmac_sha1(const char *msg, int msg_len, char *digest, const char *key, int key_len);


#endif /* __HMAC_SHA1_H__ */
