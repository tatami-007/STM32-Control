"""验证main.c的CubeMX生成区未被修改；只允许更改USER CODE内容。"""
import pathlib,re,hashlib
s=pathlib.Path('Core/Src/main.c').read_bytes().decode('utf-8')
pattern=r'(/\* USER CODE BEGIN ([^\r\n*]+) \*/)(.*?)(/\* USER CODE END \2 \*/)'
s=re.sub(pattern,lambda m:m[1]+'\n<USER:'+m[2]+'>\n'+m[4],s,flags=re.S).replace('\r\n','\n')
s=re.sub(r'(?m)^[ \t]+(?=/\* USER CODE (?:BEGIN|END))','',s) # 忽略CubeMX重新生成时的标记缩进
assert hashlib.sha256(s.encode()).hexdigest()=="104b5c98c8e2cd318965688bea600125681528f1102ca38ed498316f1904ffdc", 'CubeMX生成区变更：请核对备份，不要当作用户代码精简'
print('PASS CubeMX generated main.c preserved')
