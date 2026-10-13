"""验证main.c的CubeMX生成区未被修改；只允许更改USER CODE内容。"""
import pathlib,re,hashlib
s=pathlib.Path('Core/Src/main.c').read_bytes().decode('utf-8')
pattern=r'(/\* USER CODE BEGIN ([^\r\n*]+) \*/)(.*?)(/\* USER CODE END \2 \*/)'
s=re.sub(pattern,lambda m:m[1]+'\n<USER:'+m[2]+'>\n'+m[4],s,flags=re.S).replace('\r\n','\n')
assert hashlib.sha256(s.encode()).hexdigest()=="48261c0c94caeace8a66c639d6c8788d85474b13be000f7ef53e5417fed322cb", 'CubeMX生成区变更：请核对备份，不要当作用户代码精简'
print('PASS CubeMX generated main.c preserved')
